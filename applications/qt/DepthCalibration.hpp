#pragma once

#include <QByteArray>
#include <QString>
#include <QVector>
#include <cstdint>
#include <vector>

struct DepthIntrinsics {
  int width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  QByteArray extension;
  bool valid() const { return width > 0 && height > 0 && fx > 0 && fy > 0; }
  bool ray(int x, int y, double& rx, double& ry) const;
};

class CalibrationCapture {
 public:
  explicit CalibrationCapture(DepthIntrinsics intrinsics);
  bool add(const QByteArray& metricMm16);
  int frames() const { return frames_; }
  const DepthIntrinsics& intrinsics() const { return intrinsics_; }
  int meanMm(int x, int y) const;
 private:
  DepthIntrinsics intrinsics_;
  QVector<quint64> sums_;
  QVector<quint16> counts_;
  int frames_ = 0;
};

struct CalibrationReport {
  int samples = 0;
  double beforeRmsMm = 0;
  double afterRmsMm = 0;
  double centerDistanceM = 0;
  bool valid = false;
};

class DepthCalibrationProfile {
 public:
  static constexpr int gridWidth = 16, gridHeight = 12;
  explicit DepthCalibrationProfile(QString deviceId = {});
  bool addStation(const CalibrationCapture& capture, double tapeDistanceM,
                  CalibrationReport& report);
  CalibrationReport benchmark(const CalibrationCapture& capture) const;
  quint16 correct(int x, int y, quint16 millimeters) const;
  bool save(const QString& path) const;
  bool load(const QString& path, const QString& deviceId, int width, int height);
  void clear();
  int stationCount() const { return int(stations_.size()); }
 private:
  struct Station {
    double rawInverseCenter = 0;
    double ax = 0, by = 0, c = 0;
    std::vector<float> residual;
  };
  float gridAt(const Station& station, int x, int y) const;
  QString deviceId_;
  int width_ = 0, height_ = 0;
  std::vector<Station> stations_;
};
