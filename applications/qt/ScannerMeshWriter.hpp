#pragma once

#include <QString>
#include <vector>
#include "TsdfVolume.hpp"

class ScannerMeshWriter {
 public:
  static bool write(const QString& path,const std::vector<ScannerTriangle>& triangles,
                    QString& error);
};
