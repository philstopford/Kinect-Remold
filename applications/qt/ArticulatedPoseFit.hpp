#pragma once

#include <QJsonObject>
#include <QVector3D>
#include <QString>
#include <array>
#include <vector>

// Fits one connected, subject-sized skeleton to the observed camera-space joints.
// State is deliberately owned by one capture session and reset on device changes.
class ArticulatedPoseFit {
 public:
  QJsonObject fit(QJsonObject frame);
  void reset();
 private:
  struct Bone {
    int a, b;
    float nominal, low, high;
    std::vector<float> samples;
    float length = 0;
    bool locked = false;
  };
  struct Joint {
    QVector3D observed, position;
    float confidence = 0;
    bool valid = false;
  };
  static constexpr int count = 12;
  std::array<Bone, 12> bones_{{
      {0,1,.38f,.25f,.55f}, {6,7,.29f,.19f,.43f},
      {0,6,.47f,.32f,.65f}, {1,7,.47f,.32f,.65f},
      {0,2,.29f,.19f,.42f}, {2,4,.26f,.17f,.39f},
      {1,3,.29f,.19f,.42f}, {3,5,.26f,.17f,.39f},
      {6,8,.43f,.29f,.62f}, {8,10,.42f,.28f,.61f},
      {7,9,.43f,.29f,.62f}, {9,11,.42f,.28f,.61f}}};
  QString device_;
  double lastTime_ = 0;
  int frames_ = 0;
  std::array<QVector3D,count> previous_{};
  std::array<bool,count> previousValid_{};
};
