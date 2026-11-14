#include "LanPoseLink.hpp"

#include <QFile>
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <QSslCertificate>
#include <QSslConfiguration>
#include <QSslKey>
#include <QSslSocket>
#include <QSysInfo>
#include <chrono>
#include <cmath>

LanPoseLink::LanPoseLink(QObject* parent)
    :QObject(parent),server_(QStringLiteral("Kinect Remold observations"),
                            QWebSocketServer::SecureMode,this) {
  connect(&server_,&QWebSocketServer::newConnection,this,[this]{
    while(server_.hasPendingConnections()) {
      QWebSocket* socket=server_.nextPendingConnection();
      if(inbound_.size()>=16||socket->sslConfiguration().peerCertificate().isNull()) {
        socket->close();socket->deleteLater();continue;
      }
      inbound_.append(socket);emit peersChanged(connectedPeers());
      connect(socket,&QWebSocket::disconnected,this,[this,socket]{
        inbound_.removeAll(socket);lastSequence_.remove(socket);
        socket->deleteLater();emit peersChanged(connectedPeers());
      });
      connect(socket,&QWebSocket::textMessageReceived,this,[this,socket](const QString& text){
        receive(socket,text);
      });
    }
  });
}

LanPoseLink::~LanPoseLink() {stop();}

bool LanPoseLink::configureTls(const QString& certificate,const QString& privateKey,
                               const QString& caCertificates,QSslConfiguration& config) {
  QFile certFile(certificate),keyFile(privateKey),caFile(caCertificates);
  if(!certFile.open(QIODevice::ReadOnly)||!keyFile.open(QIODevice::ReadOnly)||
     !caFile.open(QIODevice::ReadOnly)) {
    error_="Open a certificate, private key and trusted CA file before using LAN capture";
    return false;
  }
  const QSslCertificate cert(certFile.readAll(),QSsl::Pem);
  const QByteArray keyBytes=keyFile.readAll();
  QSslKey key(keyBytes,QSsl::Ec,QSsl::Pem);
  if(key.isNull()) key=QSslKey(keyBytes,QSsl::Rsa,QSsl::Pem);
  const QList<QSslCertificate> cas=QSslCertificate::fromData(caFile.readAll(),QSsl::Pem);
  if(cert.isNull()||key.isNull()||cas.isEmpty()||!QSslSocket::supportsSsl()) {
    error_="Invalid TLS identity or trust bundle";return false;
  }
  config=QSslConfiguration::defaultConfiguration();
  config.setLocalCertificate(cert);config.setPrivateKey(key);
  config.setCaCertificates(cas);
  config.setPeerVerifyMode(QSslSocket::VerifyPeer);
  config.setProtocol(QSsl::TlsV1_2OrLater);
  return true;
}

bool LanPoseLink::startHost(quint16 port,const QString& certificate,
                            const QString& privateKey,const QString& caCertificates) {
  error_.clear();server_.close();
  QSslConfiguration config;
  if(!configureTls(certificate,privateKey,caCertificates,config))return false;
  server_.setSslConfiguration(config);
  if(!server_.listen(QHostAddress::AnyIPv4,port)) {
    error_=server_.errorString();return false;
  }
  return true;
}

bool LanPoseLink::connectPeer(const QUrl& url,const QString& certificate,
                              const QString& privateKey,const QString& caCertificates) {
  error_.clear();
  if(url.scheme()!="wss"||url.host().isEmpty()) {
    error_="Use a wss:// URL with a named host";return false;
  }
  QSslConfiguration config;
  if(!configureTls(certificate,privateKey,caCertificates,config))return false;
  auto* socket=new QWebSocket(QString(),QWebSocketProtocol::VersionLatest,this);
  socket->setSslConfiguration(config);
  outbound_.append(socket);
  connect(socket,&QWebSocket::connected,this,[this]{emit peersChanged(connectedPeers());});
  connect(socket,&QWebSocket::disconnected,this,[this,socket]{
    outbound_.removeAll(socket);lastSequence_.remove(socket);
    socket->deleteLater();emit peersChanged(connectedPeers());
  });
  connect(socket,&QWebSocket::textMessageReceived,this,[this,socket](const QString& text){
    receive(socket,text);
  });
  connect(socket,&QWebSocket::errorOccurred,this,[this,socket](QAbstractSocket::SocketError){
    emit failed(socket->errorString());
  });
  socket->open(url);return true;
}

void LanPoseLink::broadcastPose(const QByteArray& cameraPose) {
  if(outbound_.isEmpty())return;
  QJsonObject root=QJsonDocument::fromJson(cameraPose).object();
  if(root.value("space")!="camera"||root.value("device").toString().isEmpty())return;
  root.remove("tracks");
  const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const QJsonObject envelope{{"v",1},{"kind","pose-observation"},
      {"host",QSysInfo::machineHostName()},{"sensor",root.value("device")},
      {"seq",double(++sequence_)},{"tMonoNs",QString::number(ns)},
      {"clockUncertaintyNs",1000000000},
      {"calibrationRevision",QStringLiteral("unconfigured")},
      {"pose",root}};
  const QString text=QString::fromUtf8(QJsonDocument(envelope).toJson(QJsonDocument::Compact));
  for(QWebSocket* socket:outbound_)
    if(socket->state()==QAbstractSocket::ConnectedState&&socket->bytesToWrite()<1024*1024)
      socket->sendTextMessage(text);
}

void LanPoseLink::receive(QWebSocket* socket,const QString& text) {
  if(text.size()>262144) {socket->close();return;}
  const QJsonObject envelope=QJsonDocument::fromJson(text.toUtf8()).object();
  if(envelope.value("v").toInt()!=1||envelope.value("kind")!="pose-observation")return;
  const QString host=envelope.value("host").toString();
  const QString sensor=envelope.value("sensor").toString();
  const QJsonObject observation=envelope.value("pose").toObject();
  const QJsonValue sequenceValue=envelope.value("seq");
  const double sequenceNumber=sequenceValue.toDouble(-1);
  const quint64 sequence=sequenceNumber>=1&&sequenceNumber<=9007199254740991.0&&
      std::floor(sequenceNumber)==sequenceNumber?quint64(sequenceNumber):0;
  bool timeValid=false;
  const quint64 captureNs=envelope.value("tMonoNs").toString().toULongLong(&timeValid);
  const double uncertainty=envelope.value("clockUncertaintyNs").toDouble(-1);
  const QString revision=envelope.value("calibrationRevision").toString();
  if(host.isEmpty()||host.size()>128||sensor.isEmpty()||sensor.size()>128||
     sequence==0||!timeValid||captureNs==0||
     !std::isfinite(uncertainty)||uncertainty<0||uncertainty>1e12||
     revision.isEmpty()||revision.size()>128)return;
  if(observation.value("v").toInt()!=1||observation.value("space")!="camera"||
     observation.value("device").toString()!=sensor)return;
  const QJsonArray bodies=observation.value("bodies").toArray();
  if(bodies.size()>1)return;
  for(const QJsonValue& bodyValue:bodies) {
    const QJsonObject joints=bodyValue.toObject().value("joints").toObject();
    if(joints.size()>100)return;
    for(auto it=joints.begin();it!=joints.end();++it) {
      const QJsonObject joint=it.value().toObject();
      const QJsonArray p=joint.value("p").toArray();
      const double confidence=joint.value("c").toDouble(-1);
      if(p.size()!=3||!std::isfinite(confidence)||confidence<0||confidence>1)return;
      for(const QJsonValue& coordinate:p)
        if(!coordinate.isDouble()||!std::isfinite(coordinate.toDouble())||
           std::abs(coordinate.toDouble())>20)return;
    }
  }
  const QJsonObject face=observation.value("face").toObject();
  if(face.value("tracked").toBool()) {
    const double confidence=face.value("confidence").toDouble(-1);
    const QJsonArray landmarks=face.value("landmarks").toArray();
    if(!std::isfinite(confidence)||confidence<0||confidence>1||
       (!landmarks.isEmpty()&&landmarks.size()!=478)||
       face.value("blendshapes").toObject().size()>64)return;
    for(const QJsonValue& value:landmarks) {
      const QJsonArray p=value.toArray();
      if(p.size()!=3)return;
      for(const QJsonValue& coordinate:p)
        if(!coordinate.isDouble()||!std::isfinite(coordinate.toDouble())||
           std::abs(coordinate.toDouble())>20)return;
    }
  }
  const QString peer=QString::fromLatin1(socket->sslConfiguration().peerCertificate()
      .digest(QCryptographicHash::Sha256).toHex().left(8));
  if(peer.isEmpty())return;
  const QString source=host+"@"+peer+"/"+sensor;
  if(sequence<=lastSequence_[socket].value(source,0))return;
  lastSequence_[socket][source]=sequence;
  QJsonObject pose=observation;
  pose["sourceT"]=pose.value("t");
  pose["device"]=source;
  pose["host"]=host;pose["sensor"]=sensor;pose["seq"]=sequenceValue;
  pose["tMonoNs"]=QString::number(captureNs);
  pose["receivedAtMs"]=double(QDateTime::currentMSecsSinceEpoch());
  pose["clockUncertaintyNs"]=uncertainty;
  pose["calibrationRevision"]=revision;
  emit remotePose(source,QJsonDocument(pose).toJson(QJsonDocument::Compact));
}

int LanPoseLink::connectedPeers() const {
  int count=inbound_.size();
  for(const QWebSocket* socket:outbound_)
    if(socket->state()==QAbstractSocket::ConnectedState)++count;
  return count;
}
void LanPoseLink::stop() {
  server_.close();
  const auto incoming=inbound_,outgoing=outbound_;
  inbound_.clear();outbound_.clear();emit peersChanged(0);
  lastSequence_.clear();
  for(QWebSocket* socket:incoming) {socket->disconnect(this);socket->close();socket->deleteLater();}
  for(QWebSocket* socket:outgoing) {socket->disconnect(this);socket->close();socket->deleteLater();}
}
