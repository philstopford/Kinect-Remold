#pragma once

#include "MocapRig.hpp"
#include <QSaveFile>
#include <QString>

class MocapBvh {
 public:
  bool start(const QString& path);
  void append(const HumanoidPose& pose);
  bool stop();
  bool recording() const {return active_;}
  int frames() const {return frames_;}
  QString error() const {return error_;}
 private:
  void joint(QByteArray& out,int bone,int depth) const;
  static QVector3D eulerZxy(QQuaternion rotation);
  QSaveFile file_;
  int frames_=0;
  qint64 frameCountOffset_=0,frameTimeOffset_=0;
  std::array<QVector3D,HumanoidPose::count> offsets_{};
  double first_=0,last_=0;
  bool active_=false;
  QString error_;
};
