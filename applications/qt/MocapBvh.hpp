#pragma once

#include "MocapRig.hpp"
#include <QSaveFile>
#include <QString>
#include <QVector>

class MocapBvh {
 public:
  bool start(const QString& path);
  void append(const HumanoidPose& pose);
  bool stop();
  bool recording() const {return active_;}
  int frames() const {return rows_.size();}
  QString error() const {return error_;}
 private:
  void joint(QByteArray& out,int bone,int depth) const;
  static QVector3D eulerZxy(QQuaternion rotation);
  QSaveFile file_;
  QVector<QByteArray> rows_;
  std::array<QVector3D,HumanoidPose::count> offsets_{};
  double first_=0,last_=0;
  bool active_=false;
  QString error_;
};
