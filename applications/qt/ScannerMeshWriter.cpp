#include "ScannerMeshWriter.hpp"

#include <QDataStream>
#include <QFileInfo>
#include <QSaveFile>
#include <algorithm>
#include <limits>

namespace {
bool writeObj(QSaveFile& file,const std::vector<ScannerTriangle>& triangles) {
  quint64 vertex=0;
  for(const auto& triangle:triangles) {
    const QVector3D points[]{triangle.a,triangle.b,triangle.c};
    const QRgb colors[]{triangle.ca,triangle.cb,triangle.cc};
    for(int i=0;i<3;++i) {
      const QVector3D& p=points[i];const QRgb color=colors[i];
      const QByteArray line="v "+QByteArray::number(p.x(),'g',8)+" "+
          QByteArray::number(p.y(),'g',8)+" "+QByteArray::number(p.z(),'g',8)+" "+
          QByteArray::number(qRed(color)/255.0,'f',5)+" "+
          QByteArray::number(qGreen(color)/255.0,'f',5)+" "+
          QByteArray::number(qBlue(color)/255.0,'f',5)+"\n";
      if(file.write(line)!=line.size()) return false;
    }
    const QByteArray face="f "+QByteArray::number(vertex+1)+" "+
        QByteArray::number(vertex+2)+" "+QByteArray::number(vertex+3)+"\n";
    if(file.write(face)!=face.size()) return false;
    vertex+=3;
  }
  return true;
}
void vertex(QDataStream& stream,const QVector3D& p) {
  stream<<p.x()<<p.y()<<p.z();
}
bool writePly(QSaveFile& file,const std::vector<ScannerTriangle>& triangles) {
  const QByteArray header="ply\nformat binary_little_endian 1.0\n"+
      QByteArray("element vertex ")+QByteArray::number(triangles.size()*3)+
      "\nproperty float x\nproperty float y\nproperty float z\n"+
      QByteArray("property uchar red\nproperty uchar green\nproperty uchar blue\n")+
      QByteArray("element face ")+QByteArray::number(triangles.size())+
      "\nproperty list uchar int vertex_indices\nend_header\n";
  if(file.write(header)!=header.size()) return false;
  QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);
  stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
  for(const auto& t:triangles) {
    const QVector3D points[]{t.a,t.b,t.c};const QRgb colors[]{t.ca,t.cb,t.cc};
    for(int i=0;i<3;++i) {
      vertex(stream,points[i]);
      stream<<quint8(qRed(colors[i]))<<quint8(qGreen(colors[i]))<<quint8(qBlue(colors[i]));
    }
  }
  for(quint32 i=0;i<triangles.size();++i) {
    stream<<quint8(3)<<qint32(i*3)<<qint32(i*3+1)<<qint32(i*3+2);
  }
  return stream.status()==QDataStream::Ok;
}
bool writeStl(QSaveFile& file,const std::vector<ScannerTriangle>& triangles) {
  QByteArray header(80,0);const QByteArray title="Kinect Remold TSDF mesh";
  std::copy(title.begin(),title.end(),header.begin());
  if(file.write(header)!=header.size()) return false;
  QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);
  stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
  stream<<quint32(triangles.size());
  for(const auto& t:triangles) {
    QVector3D normal=QVector3D::crossProduct(t.b-t.a,t.c-t.a);
    if(normal.lengthSquared()>1e-12f) normal.normalize();
    vertex(stream,normal);vertex(stream,t.a);vertex(stream,t.b);vertex(stream,t.c);
    stream<<quint16(0);
  }
  return stream.status()==QDataStream::Ok;
}
}

bool ScannerMeshWriter::write(const QString& path,const std::vector<ScannerTriangle>& triangles,
                              QString& error) {
  error.clear();
  if(triangles.empty() || triangles.size()>std::numeric_limits<quint32>::max()/3) {
    error="Mesh is empty or too large";return false;
  }
  const QString suffix=QFileInfo(path).suffix().toLower();
  if(suffix!="obj"&&suffix!="ply"&&suffix!="stl") {
    error="Choose OBJ, PLY, or STL";return false;
  }
  QSaveFile file(path);
  if(!file.open(QIODevice::WriteOnly)) {error=file.errorString();return false;}
  const bool wrote=suffix=="obj"?writeObj(file,triangles):
      suffix=="ply"?writePly(file,triangles):writeStl(file,triangles);
  if(!wrote || !file.commit()) {
    error=file.errorString().isEmpty()?QStringLiteral("Could not finish mesh export"):file.errorString();
    return false;
  }
  return true;
}
