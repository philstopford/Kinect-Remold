#pragma once

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QMap>
#include <QString>

class MocapWebSocket : public QObject {
  Q_OBJECT
 public:
  explicit MocapWebSocket(QObject* parent=nullptr);
  bool start(quint16 port);
  void stop();
  void broadcast(const QByteArray& json);
  QString error() const {return error_;}
  quint16 port() const {return server_.serverPort();}
  int clients() const {return clients_.size();}
  quint64 messages() const {return messages_;}
 private:
  struct Client {QByteArray pending;bool open=false;};
  void onConnection();
  void onRead(QTcpSocket* socket);
  static QByteArray frame(quint8 opcode,const QByteArray& payload);
  QTcpServer server_;
  QMap<QTcpSocket*,Client> clients_;
  QString error_;
  quint64 messages_=0;
};
