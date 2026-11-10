#pragma once

#include <QByteArray>
#include <QImage>
#include <QColor>
#include <QVector3D>
#include <array>
#include <cstdint>
#include <vector>

#include "DepthCalibration.hpp"

struct ScannerPose {
  // Row-major rigid transform from camera coordinates to world coordinates.
  std::array<float, 12> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  QVector3D world(const QVector3D& point) const;
  QVector3D camera(const QVector3D& point) const;
  ScannerPose compose(const ScannerPose& other) const;
};

struct ScannerTriangle {
  QVector3D a, b, c;
  QRgb ca=qRgb(180,180,180),cb=qRgb(180,180,180),cc=qRgb(180,180,180);
};

class TsdfVolume {
 public:
  TsdfVolume(int resolution = 160, float voxelM = 0.006f, float truncationM = 0.024f);
  void reset(const QVector3D& center);
  bool integrate(const QByteArray& metricMm16, const DepthIntrinsics& camera,
                 const DepthCalibrationProfile* calibration, const ScannerPose& pose,
                 int pixelStride = 3,const QImage* color=nullptr);
  bool expandToInclude(const QVector3D& low, const QVector3D& high);
  float sample(const QVector3D& world, int minimumWeight = 1) const;
  std::vector<ScannerTriangle> mesh(int minimumWeight = 2) const;
  int observedVoxels() const { return int(touched_.size()); }
  int integratedFrames() const { return frames_; }
  float voxelSize() const { return voxel_; }
  QVector3D origin() const { return origin_; }
 private:
  struct Voxel {
    std::int16_t sdf = 0; std::uint8_t weight = 0;
    std::uint8_t r=0,g=0,b=0,colorWeight=0;
  };
  int index(int x, int y, int z) const { return x + size_ * (y + size_ * z); }
  bool inside(int x, int y, int z) const {
    return x >= 0 && y >= 0 && z >= 0 && x < size_ && y < size_ && z < size_;
  }
  QVector3D position(int x, int y, int z) const;
  float value(int x, int y, int z, int minimumWeight) const;
  QRgb colorAt(const QVector3D& world) const;
  int size_;
  float voxel_, truncation_;
  QVector3D origin_;
  std::vector<Voxel> voxels_;
  std::vector<std::uint32_t> stamps_;
  std::vector<int> touched_;
  std::uint32_t frameStamp_ = 0;
  int frames_ = 0;
};
