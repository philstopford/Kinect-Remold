#pragma once
#include <QFile>
#include <QJsonObject>
#include <QString>
#include <vector>

struct MocapClipOptions {
  QString source,face,output;
  QString bvhOutput;
  double trimStart=0,trimEnd=0,faceOffset=0,loopBlend=0;
  int smoothing=0;
  bool inPlaceLoop=true;
};

struct MocapClipResult {
  int frames=0,seamStartFrame=0,seamEndFrame=0;
  int plantedFootFrames=0;
  double seamScore=0;
  QString error,warning;
};

class MocapClipProcessor {
 public:
  static MocapClipResult process(const MocapClipOptions& options);
};

struct MocapClipIndex {
  QString source,error;
  bool compact=false;
  int frames=0;
  double firstTime=0,lastTime=0;
  std::vector<qint64> anchors;
  static MocapClipIndex build(const QString& path);
  QJsonObject frameAt(QFile& file,int index) const;
};
