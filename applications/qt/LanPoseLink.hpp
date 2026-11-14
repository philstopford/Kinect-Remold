#pragma once

#include <QObject>
#include <QWebSocket>
#include <QWebSocketServer>
#include <QHash>
#include <QUrl>

// Mutual-TLS transport for camera-space body and face observations.
class LanPoseLink : public QObject {
  Q_OBJECT
 public:
  explicit LanPoseLink(QObject* parent=nullptr);
  ~LanPoseLink() override;
  bool startHost(quint16 port,const QString& certificate,const QString& privateKey,
                 const QString& caCertificates);
  bool connectPeer(const QUrl& url,const QString& certificate,
                   const QString& privateKey,const QString& caCertificates);
  void broadcastPose(const QByteArray& cameraPose);
  void stop();
  QString error() const {return error_;}
  quint16 port() const {return server_.serverPort();}
  bool hosting() const {return server_.isListening();}
  int connectedPeers() const;
 signals:
  void remotePose(QString sensor,QByteArray cameraPose);
  void peersChanged(int count);
  void failed(QString error);
 private:
  bool configureTls(const QString& certificate,const QString& privateKey,
                    const QString& caCertificates,QSslConfiguration& config);
  void receive(QWebSocket* socket,const QString& text);
  QWebSocketServer server_;
  QList<QWebSocket*> outbound_;
  QList<QWebSocket*> inbound_;
  QString error_;
  quint64 sequence_=0;
  QHash<QWebSocket*,QHash<QString,quint64>> lastSequence_;
};
