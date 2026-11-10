#include "DepthCalibration.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
quint16 readMm(const char* data) {
  return quint16(quint8(data[0]) | (quint16(quint8(data[1])) << 8));
}
float readFloat(const QByteArray& data, int offset) {
  float value = 0;
  if (offset >= 0 && offset + 4 <= data.size()) std::memcpy(&value, data.constData() + offset, 4);
  return value;
}
quint32 readU32(const QByteArray& data, int offset) {
  quint32 value = 0;
  if (offset >= 0 && offset + 4 <= data.size()) std::memcpy(&value, data.constData() + offset, 4);
  return value;
}
bool solve3(double matrix[3][4], double& a, double& b, double& c) {
  for (int column = 0; column < 3; ++column) {
    int pivot = column;
    for (int row = column + 1; row < 3; ++row)
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) pivot = row;
    if (std::abs(matrix[pivot][column]) < 1e-12) return false;
    for (int col = column; col < 4; ++col) std::swap(matrix[column][col], matrix[pivot][col]);
    const double divisor = matrix[column][column];
    for (int col = column; col < 4; ++col) matrix[column][col] /= divisor;
    for (int row = 0; row < 3; ++row) {
      if (row == column) continue;
      const double scale = matrix[row][column];
      for (int col = column; col < 4; ++col) matrix[row][col] -= scale * matrix[column][col];
    }
  }
  a = matrix[0][3]; b = matrix[1][3]; c = matrix[2][3];
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && c > 0.05;
}
struct Plane { double a = 0, b = 0, c = 0; int samples = 0; bool valid = false; };

template <typename Corrector>
Plane fitPlane(const CalibrationCapture& capture, Corrector correct) {
  Plane result;
  const DepthIntrinsics& intrinsics = capture.intrinsics();
  if (capture.frames() < 5 || !intrinsics.valid()) return result;
  for (int iteration = 0; iteration < 5; ++iteration) {
    double xx = 0, xy = 0, x = 0, yy = 0, y = 0, n = 0, xw = 0, yw = 0, w = 0;
    int samples = 0;
    for (int py = 8; py < intrinsics.height - 8; py += 3)
      for (int px = 8; px < intrinsics.width - 8; px += 3) {
        const quint16 millimeters = correct(px, py, capture.meanMm(px, py));
        if (millimeters < 350 || millimeters > 6000) continue;
        double rx, ry;
        if (!intrinsics.ray(px, py, rx, ry)) continue;
        const double inverse = 1000.0 / millimeters;
        const double residual = inverse - (result.a * rx + result.b * ry + result.c);
        if (iteration && std::abs(residual) > 0.25) continue;
        const double weight = !iteration || std::abs(residual) <= 0.025 ? 1.0 : 0.025 / std::abs(residual);
        xx += weight * rx * rx; xy += weight * rx * ry; x += weight * rx;
        yy += weight * ry * ry; y += weight * ry; n += weight;
        xw += weight * rx * inverse; yw += weight * ry * inverse; w += weight * inverse;
        ++samples;
      }
    if (samples < 256) return {};
    double matrix[3][4]{{xx, xy, x, xw}, {xy, yy, y, yw}, {x, y, n, w}};
    if (!solve3(matrix, result.a, result.b, result.c)) return {};
    result.samples = samples;
  }
  result.valid = true;
  return result;
}

template <typename Corrector>
double planeRmsMm(const CalibrationCapture& capture, const Plane& plane, Corrector correct) {
  if (!plane.valid) return 0;
  const auto& intrinsics = capture.intrinsics();
  double squared = 0;
  int samples = 0;
  for (int py = 8; py < intrinsics.height - 8; py += 3)
    for (int px = 8; px < intrinsics.width - 8; px += 3) {
      const quint16 mm = correct(px, py, capture.meanMm(px, py));
      if (mm < 350 || mm > 6000) continue;
      double rx, ry;
      if (!intrinsics.ray(px, py, rx, ry)) continue;
      const double inverse = plane.a * rx + plane.b * ry + plane.c;
      if (inverse <= 0.05) continue;
      const double error = mm - 1000.0 / inverse;
      if (std::abs(error) > 500) continue;
      squared += error * error;
      ++samples;
    }
  return samples ? std::sqrt(squared / samples) : 0;
}
}

bool DepthIntrinsics::ray(int x, int y, double& rx, double& ry) const {
  if (!valid() || x < 0 || y < 0 || x >= width || y >= height) return false;
  const bool distortion = extension.size() >= 64 && (readU32(extension, 12) & 1u);
  const double center = distortion ? readFloat(extension, 36) : 0.5;
  const double dx = (x + center - cx) / fx, dy = (y + center - cy) / fy;
  rx = dx; ry = dy;
  if (distortion) {
    const double k1 = readFloat(extension, 16), k2 = readFloat(extension, 20);
    const double k3 = readFloat(extension, 24), p1 = readFloat(extension, 28);
    const double p2 = readFloat(extension, 32);
    for (int i = 0; i < 20; ++i) {
      const double xx = rx * rx, yy = ry * ry, rr = xx + yy;
      const double radial = 1 + k1 * rr + k2 * rr * rr + k3 * rr * rr * rr;
      const double ex = dx - (rx * radial + 2 * p1 * rx * ry + p2 * (rr + 2 * xx));
      const double ey = dy - (ry * radial + p1 * (rr + 2 * yy) + 2 * p2 * rx * ry);
      rx += ex; ry += ey;
      if (ex * ex + ey * ey < 1e-20) break;
    }
  }
  if (!std::isfinite(rx) || !std::isfinite(ry) || std::abs(rx) > 4 || std::abs(ry) > 4) {
    rx = dx; ry = dy;
  }
  return true;
}

CalibrationCapture::CalibrationCapture(DepthIntrinsics intrinsics) : intrinsics_(std::move(intrinsics)),
    sums_(intrinsics_.width * intrinsics_.height, 0),
    counts_(intrinsics_.width * intrinsics_.height, 0) {}

bool CalibrationCapture::add(const QByteArray& metricMm16) {
  if (!intrinsics_.valid() || metricMm16.size() != qsizetype(intrinsics_.width) * intrinsics_.height * 2 ||
      frames_ >= 120) return false;
  for (qsizetype i = 0; i < sums_.size(); ++i) {
    const quint16 mm = readMm(metricMm16.constData() + i * 2);
    if (mm >= 350 && mm <= 6000) { sums_[i] += mm; ++counts_[i]; }
  }
  ++frames_;
  return true;
}

int CalibrationCapture::meanMm(int x, int y) const {
  if (x < 0 || y < 0 || x >= intrinsics_.width || y >= intrinsics_.height) return 0;
  const int index = y * intrinsics_.width + x;
  if (counts_[index] < std::max(3, frames_ * 3 / 5)) return 0;
  return int(sums_[index] / counts_[index]);
}

DepthCalibrationProfile::DepthCalibrationProfile(QString deviceId) : deviceId_(std::move(deviceId)) {}
void DepthCalibrationProfile::clear() { stations_.clear(); width_ = height_ = 0; }

float DepthCalibrationProfile::gridAt(const Station& station, int x, int y) const {
  if (station.residual.size() != gridWidth * gridHeight || width_ <= 0 || height_ <= 0) return 0;
  const double gx = std::clamp(x * (gridWidth - 1.0) / std::max(1, width_ - 1), 0.0, double(gridWidth - 1));
  const double gy = std::clamp(y * (gridHeight - 1.0) / std::max(1, height_ - 1), 0.0, double(gridHeight - 1));
  const int x0 = std::min(int(gx), gridWidth - 2), y0 = std::min(int(gy), gridHeight - 2);
  const float fx = float(gx - x0), fy = float(gy - y0);
  const int a = y0 * gridWidth + x0, b = a + gridWidth;
  return (1 - fy) * ((1 - fx) * station.residual[size_t(a)] + fx * station.residual[size_t(a + 1)]) +
         fy * ((1 - fx) * station.residual[size_t(b)] + fx * station.residual[size_t(b + 1)]);
}

quint16 DepthCalibrationProfile::correct(int x, int y, quint16 millimeters) const {
  if (!millimeters || stations_.empty() || x < 0 || y < 0 || x >= width_ || y >= height_) return millimeters;
  const double inverse = 1000.0 / millimeters;
  const Station* lo = &stations_.front();
  const Station* hi = &stations_.back();
  for (size_t i = 1; i < stations_.size(); ++i) {
    if (inverse <= stations_[i].rawInverseCenter) {
      lo = &stations_[i - 1]; hi = &stations_[i]; break;
    }
  }
  if (inverse <= stations_.front().rawInverseCenter) lo = hi = &stations_.front();
  else if (inverse >= stations_.back().rawInverseCenter) lo = hi = &stations_.back();
  const double blend = lo == hi || hi->rawInverseCenter - lo->rawInverseCenter < 1e-8 ? 0 :
      std::clamp((inverse - lo->rawInverseCenter) /
      (hi->rawInverseCenter - lo->rawInverseCenter), 0.0, 1.0);
  const double residual = (1 - blend) * gridAt(*lo, x, y) + blend * gridAt(*hi, x, y);
  const double corrected = inverse - residual;
  if (!std::isfinite(corrected) || corrected <= 0.05) return millimeters;
  return quint16(std::clamp(std::lround(1000.0 / corrected), 1l, 10000l));
}

bool DepthCalibrationProfile::addStation(const CalibrationCapture& capture, double tapeDistanceM,
                                         CalibrationReport& report) {
  const auto& intrinsics = capture.intrinsics();
  if (!intrinsics.valid() || (width_ && (width_ != intrinsics.width || height_ != intrinsics.height)) ||
      stations_.size() >= 12 || !std::isfinite(tapeDistanceM) || tapeDistanceM < 0 ||
      (tapeDistanceM > 0 && (tapeDistanceM < 0.35 || tapeDistanceM > 6.0))) return false;
  const auto identity = [](int, int, int mm) { return quint16(mm); };
  const Plane plane = fitPlane(capture, identity);
  if (!plane.valid) return false;
  width_ = intrinsics.width; height_ = intrinsics.height;
  Station station;
  station.rawInverseCenter = plane.c;
  station.ax = plane.a; station.by = plane.b;
  station.c = tapeDistanceM > 0 ? 1.0 / tapeDistanceM : plane.c;
  station.residual.resize(gridWidth * gridHeight, 0);
  std::vector<double> sums(station.residual.size(), 0);
  std::vector<int> counts(station.residual.size(), 0);
  for (int y = 0; y < height_; y += 2) for (int x = 0; x < width_; x += 2) {
    const int mm = capture.meanMm(x, y);
    if (mm < 350 || mm > 6000) continue;
    double rx, ry;
    if (!intrinsics.ray(x, y, rx, ry)) continue;
    const double residual = 1000.0 / mm - (station.ax * rx + station.by * ry + station.c);
    if (!std::isfinite(residual) || std::abs(residual) > 0.5) continue;
    const int gx = std::clamp(x * gridWidth / width_, 0, gridWidth - 1);
    const int gy = std::clamp(y * gridHeight / height_, 0, gridHeight - 1);
    const int index = gy * gridWidth + gx;
    sums[size_t(index)] += residual; ++counts[size_t(index)];
  }
  for (int y = 0; y < gridHeight; ++y) for (int x = 0; x < gridWidth; ++x) {
    const int index = y * gridWidth + x;
    if (counts[size_t(index)] >= 8) station.residual[size_t(index)] = float(sums[size_t(index)] / counts[size_t(index)]);
    else {
      int closest = -1, distance = std::numeric_limits<int>::max();
      for (int j = 0; j < gridHeight; ++j) for (int i = 0; i < gridWidth; ++i) {
        const int candidate = j * gridWidth + i;
        if (counts[size_t(candidate)] < 8) continue;
        const int squared = (i - x) * (i - x) + (j - y) * (j - y);
        if (squared < distance) { distance = squared; closest = candidate; }
      }
      if (closest < 0) return false;
      station.residual[size_t(index)] = float(sums[size_t(closest)] / counts[size_t(closest)]);
    }
  }
  const auto previousStations = stations_;
  stations_.push_back(std::move(station));
  std::sort(stations_.begin(), stations_.end(), [](const Station& a, const Station& b) {
    return a.rawInverseCenter < b.rawInverseCenter;
  });
  report = benchmark(capture);
  if (report.valid) return true;
  // A failed station must not change the profile that can later be saved.
  stations_ = previousStations;
  if (stations_.empty()) { width_ = 0; height_ = 0; }
  return false;
}

CalibrationReport DepthCalibrationProfile::benchmark(const CalibrationCapture& capture) const {
  CalibrationReport report;
  if (!capture.intrinsics().valid() || capture.intrinsics().width != width_ ||
      capture.intrinsics().height != height_) return report;
  const auto identity = [](int, int, int mm) { return quint16(mm); };
  const Plane before = fitPlane(capture, identity);
  const Plane after = fitPlane(capture, [this](int x, int y, int mm) {
    return correct(x, y, quint16(mm));
  });
  if (!before.valid || !after.valid) return report;
  report.samples = std::min(before.samples, after.samples);
  report.beforeRmsMm = planeRmsMm(capture, before, identity);
  report.afterRmsMm = planeRmsMm(capture, after, [this](int x, int y, int mm) {
    return correct(x, y, quint16(mm));
  });
  report.centerDistanceM = 1.0 / after.c;
  report.valid = std::isfinite(report.beforeRmsMm) && std::isfinite(report.afterRmsMm) &&
      report.samples >= 256;
  return report;
}

bool DepthCalibrationProfile::save(const QString& path) const {
  if (deviceId_.isEmpty() || width_ <= 0 || height_ <= 0 || stations_.empty()) return false;
  QJsonObject root{{"version", 1}, {"deviceId", deviceId_}, {"width", width_}, {"height", height_}};
  QJsonArray stations;
  for (const Station& station : stations_) {
    QJsonArray grid;
    for (float value : station.residual) grid.append(double(value));
    stations.append(QJsonObject{{"rawInverseCenter", station.rawInverseCenter},
        {"ax", station.ax}, {"by", station.by}, {"c", station.c}, {"grid", grid}});
  }
  root["stations"] = stations;
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) return false;
  const QByteArray data = QJsonDocument(root).toJson(QJsonDocument::Compact);
  return file.write(data) == data.size() && file.commit();
}

bool DepthCalibrationProfile::load(const QString& path, const QString& deviceId, int width, int height) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) return false;
  QJsonParseError error{};
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) return false;
  const QJsonObject root = document.object();
  if (root.value("version").toInt() != 1 || root.value("deviceId").toString() != deviceId ||
      root.value("width").toInt() != width || root.value("height").toInt() != height) return false;
  const QJsonArray saved = root.value("stations").toArray();
  if (saved.isEmpty() || saved.size() > 12) return false;
  std::vector<Station> parsed;
  for (const auto& entry : saved) {
    if (!entry.isObject()) return false;
    const QJsonObject object = entry.toObject();
    Station station;
    station.rawInverseCenter = object.value("rawInverseCenter").toDouble();
    station.ax = object.value("ax").toDouble(); station.by = object.value("by").toDouble();
    station.c = object.value("c").toDouble();
    const QJsonArray grid = object.value("grid").toArray();
    if (grid.size() != gridWidth * gridHeight || !std::isfinite(station.rawInverseCenter) ||
        station.rawInverseCenter <= 0 || !std::isfinite(station.ax) || !std::isfinite(station.by) ||
        !std::isfinite(station.c) || station.c <= 0) return false;
    for (const auto& cell : grid) {
      const double value = cell.toDouble(std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(value) || std::abs(value) > 0.5) return false;
      station.residual.push_back(float(value));
    }
    parsed.push_back(std::move(station));
  }
  std::sort(parsed.begin(), parsed.end(), [](const Station& a, const Station& b) {
    return a.rawInverseCenter < b.rawInverseCenter;
  });
  deviceId_ = deviceId; width_ = width; height_ = height; stations_ = std::move(parsed);
  return true;
}
