#pragma once

#include <QByteArray>
#include <QHostAddress>
#include <QJsonObject>
#include <QString>
#include <QUdpSocket>
#include "MocapRig.hpp"

class MocapNetwork {
 public:
  void setJsonTarget(QHostAddress host,quint16 port) {jsonHost_=host;jsonPort_=port;}
  void setOscTarget(QHostAddress host,quint16 port) {oscHost_=host;oscPort_=port;}
  void setVmcTarget(QHostAddress host,quint16 port) {vmcHost_=host;vmcPort_=port;}
  void setFaceCapTarget(QHostAddress host,quint16 port) {faceHost_=host;facePort_=port;}
  void setJsonEnabled(bool enabled) {jsonEnabled_=enabled;}
  void setOscEnabled(bool enabled) {oscEnabled_=enabled;}
  void setVmcEnabled(bool enabled) {vmcEnabled_=enabled;}
  void setFaceCapEnabled(bool enabled) {faceCapEnabled_=enabled;}
  bool send(const QByteArray& poseJson,const HumanoidPose* rig=nullptr,
            const QByteArray& jsonOutput={});
  QString error() const {return error_;}
  quint64 jsonPackets() const {return jsonPackets_;}
  quint64 oscPackets() const {return oscPackets_;}
  quint64 vmcPackets() const {return vmcPackets_;}
  quint64 facePackets() const {return facePackets_;}
  static QByteArray encodeOsc(const QByteArray& poseJson,const HumanoidPose* rig=nullptr);
  static QByteArray encodeVmc(const HumanoidPose& rig);
  static QByteArray encodeFaceCap(const QJsonObject& face);
  static QByteArray encodeVmcFace(const QJsonObject& face);
 private:
  QUdpSocket socket_;
  QHostAddress jsonHost_=QHostAddress::LocalHost,oscHost_=QHostAddress::LocalHost,
      vmcHost_=QHostAddress::LocalHost,faceHost_=QHostAddress::LocalHost;
  quint16 jsonPort_=39541,oscPort_=39542,vmcPort_=39539,facePort_=39543;
  bool jsonEnabled_=false,oscEnabled_=false,vmcEnabled_=false,faceCapEnabled_=false;
  quint64 jsonPackets_=0,oscPackets_=0,vmcPackets_=0,facePackets_=0;
  QString error_;
};
