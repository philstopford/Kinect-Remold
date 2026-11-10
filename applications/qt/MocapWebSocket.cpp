#include "MocapWebSocket.hpp"

#include <QCryptographicHash>
#include <QHostAddress>

MocapWebSocket::MocapWebSocket(QObject* parent):QObject(parent) {
  connect(&server_,&QTcpServer::newConnection,this,&MocapWebSocket::onConnection);
}

bool MocapWebSocket::start(quint16 port) {
  stop();error_.clear();messages_=0;
  if(!server_.listen(QHostAddress::LocalHost,port)) {
    error_=server_.errorString();return false;
  }
  return true;
}
void MocapWebSocket::stop() {
  server_.close();
  for(auto socket:clients_.keys()) {socket->disconnectFromHost();socket->deleteLater();}
  clients_.clear();
}
void MocapWebSocket::onConnection() {
  while(server_.hasPendingConnections()) {
    QTcpSocket* socket=server_.nextPendingConnection();
    if(clients_.size()>=16) {socket->disconnectFromHost();socket->deleteLater();continue;}
    clients_.insert(socket,{});
    connect(socket,&QTcpSocket::readyRead,this,[this,socket]{onRead(socket);});
    connect(socket,&QTcpSocket::disconnected,this,[this,socket]{
      if(!clients_.isEmpty()) clients_.remove(socket);
      socket->deleteLater();
    });
  }
}
void MocapWebSocket::onRead(QTcpSocket* socket) {
  if(clients_.isEmpty()) return;
  auto it=clients_.find(socket);if(it==clients_.end()) return;
  Client& client=it.value();client.pending+=socket->readAll();
  if(client.pending.size()>8192) {socket->disconnectFromHost();return;}
  if(!client.open) {
    const int end=client.pending.indexOf("\r\n\r\n");
    if(end<0) return;
    const QByteArray request=client.pending.left(end+4);
    client.pending.remove(0,end+4);
    if(!request.startsWith("GET ")||!request.toLower().contains("upgrade: websocket")||
       !request.toLower().contains("connection: upgrade")||
       !request.toLower().contains("sec-websocket-version: 13")) {
      socket->disconnectFromHost();return;
    }
    QByteArray key;
    for(const QByteArray& line:request.split('\n')) {
      if(line.trimmed().toLower().startsWith("sec-websocket-key:"))
        key=line.mid(line.indexOf(':')+1).trimmed();
    }
    if(key.size()<16||key.size()>64) {socket->disconnectFromHost();return;}
    const QByteArray accept=QCryptographicHash::hash(
        key+"258EAFA5-E914-47DA-95CA-C5AB0DC85B11",QCryptographicHash::Sha1).toBase64();
    socket->write("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                  "Connection: Upgrade\r\nSec-WebSocket-Accept: "+accept+"\r\n\r\n");
    client.open=true;
  }
  // Receiving application data is not part of this output-only endpoint. A
  // close frame or malformed/unbounded input closes the client cleanly.
  if(client.pending.size()>=2 && (quint8(client.pending[0])&15u)==8u)
    socket->disconnectFromHost();
  else client.pending.clear();
}
QByteArray MocapWebSocket::frame(quint8 opcode,const QByteArray& payload) {
  QByteArray result;result.append(char(0x80u|opcode));
  const quint64 size=quint64(payload.size());
  if(size<126) result.append(char(size));
  else if(size<65536) {
    result.append(char(126));result.append(char(size>>8));result.append(char(size));
  } else {
    result.append(char(127));
    for(int bits=56;bits>=0;bits-=8) result.append(char(size>>bits));
  }
  result+=payload;return result;
}
void MocapWebSocket::broadcast(const QByteArray& json) {
  if(!server_.isListening()) return;
  const QByteArray packet=frame(1,json);
  for(auto it=clients_.begin();it!=clients_.end();++it) {
    QTcpSocket* socket=it.key();
    if(!it.value().open||socket->bytesToWrite()>1024*1024) continue;
    if(socket->write(packet)==packet.size()) ++messages_;
  }
}
