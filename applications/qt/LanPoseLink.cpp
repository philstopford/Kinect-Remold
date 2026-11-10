#include "LanPoseLink.hpp"

#include <QFile>
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
  if(inbound_.isEmpty()&&outbound_.isEmpty())return;
  const QJsonObject root=QJsonDocument::fromJson(cameraPose).object();
  const QJsonArray bodies=root.value("bodies").toArray();
  if(root.value("space")!="camera"||bodies.isEmpty())return;
  const QJsonObject body=bodies.first().toObject();
  QJsonArray joints;
  const QJsonObject source=body.value("joints").toObject();
  for(auto it=source.begin();it!=source.end();++it) {
    const QJsonObject value=it.value().toObject();
    joints.append(QJsonObject{{"name",it.key()},{"p",value.value("p")},
                              {"c",value.value("c")},{"state","tracked"}});
  }
  const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const QJsonObject envelope{{"v",1},{"kind","body-observation"},
      {"host",QSysInfo::machineHostName()},{"sensor",root.value("device")},
      {"seq",double(++sequence_)},{"tMonoNs",QString::number(ns)},
      {"clockUncertaintyNs",1000000000},
      {"calibrationRevision",QStringLiteral("unconfigured")},
      {"subject",QString::number(body.value("id").toInt())},{"joints",joints}};
  const QString text=QString::fromUtf8(QJsonDocument(envelope).toJson(QJsonDocument::Compact));
  for(QWebSocket* socket:inbound_)
    if(socket->state()==QAbstractSocket::ConnectedState&&socket->bytesToWrite()<1024*1024)
      socket->sendTextMessage(text);
  for(QWebSocket* socket:outbound_)
    if(socket->state()==QAbstractSocket::ConnectedState&&socket->bytesToWrite()<1024*1024)
      socket->sendTextMessage(text);
}

void LanPoseLink::receive(QWebSocket* socket,const QString& text) {
  if(text.size()>262144) {socket->close();return;}
  const QJsonObject envelope=QJsonDocument::fromJson(text.toUtf8()).object();
  if(envelope.value("v").toInt()!=1||envelope.value("kind")!="body-observation")return;
  const QString host=envelope.value("host").toString();
  const QString sensor=envelope.value("sensor").toString();
  const QJsonArray observations=envelope.value("joints").toArray();
  const QJsonValue sequenceValue=envelope.value("seq");
  const double sequenceNumber=sequenceValue.toDouble(-1);
  const quint64 sequence=sequenceNumber>=1&&sequenceNumber<=9007199254740991.0&&
      std::floor(sequenceNumber)==sequenceNumber?quint64(sequenceNumber):0;
  bool timeValid=false;
  const quint64 captureNs=envelope.value("tMonoNs").toString().toULongLong(&timeValid);
  const double uncertainty=envelope.value("clockUncertaintyNs").toDouble(-1);
  const QString revision=envelope.value("calibrationRevision").toString();
  if(host.isEmpty()||host.size()>128||sensor.isEmpty()||sensor.size()>128||
     observations.size()>100||sequence==0||!timeValid||captureNs==0||
     !std::isfinite(uncertainty)||uncertainty<0||uncertainty>1e12||
     revision.isEmpty()||revision.size()>128)return;
  QJsonObject joints;
  for(const QJsonValue& entry:observations) {
    const QJsonObject j=entry.toObject();const QString name=j.value("name").toString();
    const QJsonArray p=j.value("p").toArray();
    const double c=j.value("c").toDouble();
    if(name.isEmpty()||name.size()>64||p.size()!=3||!std::isfinite(c)||c<0||c>1)continue;
    bool finite=true;for(const QJsonValue& coordinate:p)
      finite&=coordinate.isDouble()&&std::isfinite(coordinate.toDouble())&&
          std::abs(coordinate.toDouble())<20;
    if(!finite)continue;
    joints[name]=QJsonObject{{"p",p},{"c",c},{"s",2}};
  }
  const QJsonObject body{{"id",1},{"tracked",!joints.isEmpty()},
                         {"confidence",.5},{"joints",joints}};
  const QString peer=QString::fromLatin1(socket->sslConfiguration().peerCertificate()
      .digest(QCryptographicHash::Sha256).toHex().left(8));
  if(peer.isEmpty())return;
  const QString source=host+"@"+peer+"/"+sensor;
  if(sequence<=lastSequence_[socket].value(source,0))return;
  lastSequence_[socket][source]=sequence;
  const QJsonObject pose{{"v",1},{"device",source},{"space","camera"},
                         {"tracked",!joints.isEmpty()},
                         {"bodies",QJsonArray{body}},
                         {"host",host},{"sensor",sensor},{"seq",sequenceValue},
                         {"tMonoNs",QString::number(captureNs)},
                         {"clockUncertaintyNs",uncertainty},
                         {"calibrationRevision",revision}};
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
