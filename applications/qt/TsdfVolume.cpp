#include "TsdfVolume.hpp"
#include "RgbdRegistration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
quint16 readMm(const QByteArray& data, int index) {
  const auto* p = reinterpret_cast<const quint8*>(data.constData() + index * 2);
  return quint16(p[0] | (quint16(p[1]) << 8));
}
QVector3D interpolate(const QVector3D& a, const QVector3D& b, float va, float vb) {
  const float denominator = va - vb;
  const float t = std::abs(denominator) < 1e-6f ? .5f : std::clamp(va / denominator, 0.0f, 1.0f);
  return a + (b - a) * t;
}
}

QVector3D ScannerPose::world(const QVector3D& p) const {
  return {m[0]*p.x()+m[1]*p.y()+m[2]*p.z()+m[3],
          m[4]*p.x()+m[5]*p.y()+m[6]*p.z()+m[7],
          m[8]*p.x()+m[9]*p.y()+m[10]*p.z()+m[11]};
}
QVector3D ScannerPose::camera(const QVector3D& p) const {
  const float x=p.x()-m[3], y=p.y()-m[7], z=p.z()-m[11];
  return {m[0]*x+m[4]*y+m[8]*z, m[1]*x+m[5]*y+m[9]*z, m[2]*x+m[6]*y+m[10]*z};
}

ScannerPose ScannerPose::compose(const ScannerPose& other) const {
  ScannerPose result;
  for (int row=0; row<3; ++row) {
    for (int col=0; col<3; ++col) {
      float sum=0;
      for (int k=0;k<3;++k) sum+=m[row*4+k]*other.m[k*4+col];
      result.m[row*4+col]=sum;
    }
    result.m[row*4+3]=m[row*4+3];
    for (int k=0;k<3;++k) result.m[row*4+3]+=m[row*4+k]*other.m[k*4+3];
  }
  return result;
}

TsdfVolume::TsdfVolume(int resolution, float voxelM, float truncationM)
    : size_(std::clamp(resolution, 16, 320)), voxel_(std::clamp(voxelM, .001f, .05f)),
      truncation_(std::max(truncationM, voxel_ * 3)),
      voxels_(size_t(size_) * size_ * size_), stamps_(voxels_.size()) {
  reset({0, 0, .8f});
}

void TsdfVolume::reset(const QVector3D& center) {
  std::fill(voxels_.begin(), voxels_.end(), Voxel{});
  std::fill(stamps_.begin(), stamps_.end(), 0);
  touched_.clear(); frameStamp_ = 0; frames_ = 0;
  origin_ = center - QVector3D(size_*voxel_*.5f, size_*voxel_*.5f, size_*voxel_*.5f);
}

bool TsdfVolume::expandToInclude(const QVector3D& low,const QVector3D& high) {
  if (touched_.empty()) return false;
  QVector3D minPoint=low,maxPoint=high;
  for (int id:touched_) {
    const int z=id/(size_*size_),rem=id-z*size_*size_,y=rem/size_,x=rem-y*size_;
    const QVector3D p=position(x,y,z);
    minPoint.setX(std::min(minPoint.x(),p.x()));
    minPoint.setY(std::min(minPoint.y(),p.y()));
    minPoint.setZ(std::min(minPoint.z(),p.z()));
    maxPoint.setX(std::max(maxPoint.x(),p.x()));
    maxPoint.setY(std::max(maxPoint.y(),p.y()));
    maxPoint.setZ(std::max(maxPoint.z(),p.z()));
  }
  const QVector3D margin(voxel_*8,voxel_*8,voxel_*8);
  const QVector3D end=origin_+QVector3D((size_-1)*voxel_,(size_-1)*voxel_,(size_-1)*voxel_);
  if (minPoint.x()>=origin_.x()+margin.x() && minPoint.y()>=origin_.y()+margin.y() &&
      minPoint.z()>=origin_.z()+margin.z() && maxPoint.x()<=end.x()-margin.x() &&
      maxPoint.y()<=end.y()-margin.y() && maxPoint.z()<=end.z()-margin.z()) return false;
  const QVector3D span=maxPoint-minPoint;
  const float extent=std::max({span.x(),span.y(),span.z()});
  const float newVoxel=std::max(voxel_,extent*1.18f/(size_-4));
  if (newVoxel>.05f) return false;
  const float oldVoxel=voxel_,oldTruncation=truncation_;
  const QVector3D oldOrigin=origin_;
  auto oldVoxels=std::move(voxels_);
  auto oldTouched=std::move(touched_);
  voxel_=newVoxel;truncation_=std::max(truncation_,voxel_*4);
  origin_=(minPoint+maxPoint)*.5f-QVector3D(size_*voxel_*.5f,size_*voxel_*.5f,size_*voxel_*.5f);
  voxels_.assign(oldVoxels.size(),{});
  touched_.clear();std::fill(stamps_.begin(),stamps_.end(),0);frameStamp_=0;
  for (int id:oldTouched) {
    const int z=id/(size_*size_),rem=id-z*size_*size_,y=rem/size_,x=rem-y*size_;
    const QVector3D p=oldOrigin+QVector3D(x*oldVoxel,y*oldVoxel,z*oldVoxel);
    const QVector3D grid=(p-origin_)/voxel_;
    const int nx=int(std::lround(grid.x())),ny=int(std::lround(grid.y())),nz=int(std::lround(grid.z()));
    if(!inside(nx,ny,nz)) continue;
    const int dst=index(nx,ny,nz);
    if(voxels_[size_t(dst)].weight>=oldVoxels[size_t(id)].weight) continue;
    if(!voxels_[size_t(dst)].weight) touched_.push_back(dst);
    Voxel v=oldVoxels[size_t(id)];
    v.sdf=std::int16_t(std::lround(std::clamp(v.sdf/32767.0f*oldTruncation/truncation_,
                                           -1.0f,1.0f)*32767));
    voxels_[size_t(dst)]=v;
  }
  return true;
}

QVector3D TsdfVolume::position(int x, int y, int z) const {
  return origin_ + QVector3D(x*voxel_, y*voxel_, z*voxel_);
}

bool TsdfVolume::integrate(const QByteArray& depth, const DepthIntrinsics& camera,
    const DepthCalibrationProfile* calibration, const ScannerPose& pose, int pixelStride,
    const QImage* color) {
  if (!camera.valid() || depth.size() != qsizetype(camera.width)*camera.height*2 ||
      pixelStride < 1 || pixelStride > 16) return false;
  if (++frameStamp_ == 0) { std::fill(stamps_.begin(), stamps_.end(), 0); frameStamp_ = 1; }
  const int radius = std::max(1, int(std::ceil(truncation_ / voxel_)));
  int updated = 0;
  for (int py = pixelStride; py < camera.height-pixelStride; py += pixelStride)
    for (int px = pixelStride; px < camera.width-pixelStride; px += pixelStride) {
      quint16 mm = readMm(depth, py*camera.width+px);
      if (calibration) mm = calibration->correct(px, py, mm);
      if (mm < 350 || mm > 6000) continue;
      double rx = 0, ry = 0;
      if (!camera.ray(px, py, rx, ry)) continue;
      QRgb sampleColor=0;
      if(color&&!color->isNull()) {
        float u=0,v=0;
        if(RgbdRegistration::projectKinectOne(camera,px,py,mm,color->width(),color->height(),u,v)&&
           u>=0&&v>=0&&u<color->width()&&v<color->height())
          sampleColor=color->pixel(std::clamp(int(std::lround(u)),0,color->width()-1),
                                   std::clamp(int(std::lround(v)),0,color->height()-1));
      }
      const float z = mm*.001f;
      const QVector3D surface = pose.world(QVector3D(float(rx)*z, float(ry)*z, z));
      const QVector3D grid = (surface-origin_)/voxel_;
      const int gx = int(std::lround(grid.x())), gy = int(std::lround(grid.y())),
                gz = int(std::lround(grid.z()));
      for (int dz=-radius; dz<=radius; ++dz) for (int dy=-radius; dy<=radius; ++dy)
        for (int dx=-radius; dx<=radius; ++dx) {
          const int vx=gx+dx, vy=gy+dy, vz=gz+dz;
          if (!inside(vx,vy,vz)) continue;
          const int id=index(vx,vy,vz);
          if (stamps_[size_t(id)] == frameStamp_) continue;
          stamps_[size_t(id)] = frameStamp_;
          const QVector3D candidate = pose.camera(position(vx,vy,vz));
          if (candidate.z() <= .05f) continue;
          const int u = int(std::lround(camera.fx*candidate.x()/candidate.z()+camera.cx));
          const int v = int(std::lround(camera.fy*candidate.y()/candidate.z()+camera.cy));
          if (u<0 || v<0 || u>=camera.width || v>=camera.height) continue;
          quint16 observed = readMm(depth, v*camera.width+u);
          if (calibration) observed=calibration->correct(u,v,observed);
          if (observed<350 || observed>6000) continue;
          const float sdf = observed*.001f-candidate.z();
          if (std::abs(sdf)>truncation_) continue;
          Voxel& voxel=voxels_[size_t(id)];
          if (!voxel.weight) touched_.push_back(id);
          const float old = voxel.weight ? voxel.sdf/32767.0f : 0;
          const float measurement = std::clamp(sdf/truncation_, -1.0f, 1.0f);
          const int weight = std::min(int(voxel.weight)+1, 255);
          voxel.sdf=std::int16_t(std::lround(std::clamp((old*voxel.weight+measurement)/
                                               (voxel.weight+1), -1.0f, 1.0f)*32767));
          voxel.weight=std::uint8_t(weight);
          if(sampleColor&&std::abs(sdf)<voxel_*2) {
            const int cw=std::min(int(voxel.colorWeight)+1,255);
            voxel.r=std::uint8_t((int(voxel.r)*voxel.colorWeight+qRed(sampleColor))/cw);
            voxel.g=std::uint8_t((int(voxel.g)*voxel.colorWeight+qGreen(sampleColor))/cw);
            voxel.b=std::uint8_t((int(voxel.b)*voxel.colorWeight+qBlue(sampleColor))/cw);
            voxel.colorWeight=std::uint8_t(cw);
          }
          ++updated;
        }
    }
  if (updated) ++frames_;
  return updated>0;
}

float TsdfVolume::value(int x, int y, int z, int minimumWeight) const {
  if (!inside(x,y,z)) return std::numeric_limits<float>::quiet_NaN();
  const Voxel& voxel=voxels_[size_t(index(x,y,z))];
  return voxel.weight>=minimumWeight ? voxel.sdf/32767.0f : std::numeric_limits<float>::quiet_NaN();
}

float TsdfVolume::sample(const QVector3D& world, int minimumWeight) const {
  const QVector3D grid=(world-origin_)/voxel_;
  const int x=int(std::floor(grid.x())), y=int(std::floor(grid.y())), z=int(std::floor(grid.z()));
  if (!inside(x,y,z) || !inside(x+1,y+1,z+1)) return std::numeric_limits<float>::quiet_NaN();
  const float tx=grid.x()-x, ty=grid.y()-y, tz=grid.z()-z;
  float sum=0, weights=0;
  for (int dz=0;dz<2;++dz) for (int dy=0;dy<2;++dy) for (int dx=0;dx<2;++dx) {
    const float v=value(x+dx,y+dy,z+dz,minimumWeight);
    if (!std::isfinite(v)) continue;
    const float w=(dx?tx:1-tx)*(dy?ty:1-ty)*(dz?tz:1-tz);
    sum+=w*v; weights+=w;
  }
  return weights>=.45f ? sum/weights : std::numeric_limits<float>::quiet_NaN();
}

QRgb TsdfVolume::colorAt(const QVector3D& world) const {
  const QVector3D grid=(world-origin_)/voxel_;
  const int x=int(std::lround(grid.x())),y=int(std::lround(grid.y())),
            z=int(std::lround(grid.z()));
  if(!inside(x,y,z))return qRgb(180,180,180);
  const Voxel& voxel=voxels_[size_t(index(x,y,z))];
  return voxel.colorWeight?qRgb(voxel.r,voxel.g,voxel.b):qRgb(180,180,180);
}

std::vector<ScannerTriangle> TsdfVolume::mesh(int minimumWeight) const {
  std::vector<ScannerTriangle> triangles;
  if (touched_.empty()) return triangles;
  // Marching tetrahedra uses the same eight cube corners as marching cubes,
  // with a fixed diagonal so adjacent cells agree on shared faces.
  constexpr int corners[8][3]{{0,0,0},{1,0,0},{1,1,0},{0,1,0},
                               {0,0,1},{1,0,1},{1,1,1},{0,1,1}};
  constexpr int tetra[6][4]{{0,5,1,6},{0,1,2,6},{0,2,3,6},
                             {0,3,7,6},{0,7,4,6},{0,4,5,6}};
  std::vector<std::uint8_t> visited(voxels_.size());
  for (int touched : touched_) {
    const int z=touched/(size_*size_), rem=touched-z*size_*size_;
    const int y=rem/size_, x=rem-y*size_;
    for (int dz=-1;dz<=0;++dz) for (int dy=-1;dy<=0;++dy) for (int dx=-1;dx<=0;++dx) {
      const int cx=x+dx,cy=y+dy,cz=z+dz;
      if (!inside(cx,cy,cz) || !inside(cx+1,cy+1,cz+1)) continue;
      const int cell=index(cx,cy,cz);
      if (visited[size_t(cell)]) continue;
      visited[size_t(cell)]=1;
      QVector3D p[8]; float s[8];
      int below=0, valid=0;
      for (int k=0;k<8;++k) {
        const int vx=cx+corners[k][0],vy=cy+corners[k][1],vz=cz+corners[k][2];
        p[k]=position(vx,vy,vz); s[k]=value(vx,vy,vz,minimumWeight);
        valid+=std::isfinite(s[k]); below+=std::isfinite(s[k])&&s[k]<0;
      }
      if (valid!=8 || below==0 || below==8) continue;
      for (const auto& t:tetra) {
        int positive[4],negative[4],np=0,nn=0;
        for (int j=0;j<4;++j) (s[t[j]]>=0?positive[np++]:negative[nn++])=t[j];
        if (np==0 || nn==0) continue;
        if (np==1 || nn==1) {
          const int lone=np==1?positive[0]:negative[0];
          const int* others=np==1?negative:positive;
          const QVector3D a=interpolate(p[lone],p[others[0]],s[lone],s[others[0]]);
          const QVector3D b=interpolate(p[lone],p[others[1]],s[lone],s[others[1]]);
          const QVector3D c=interpolate(p[lone],p[others[2]],s[lone],s[others[2]]);
          triangles.push_back(np==1?ScannerTriangle{a,b,c}:ScannerTriangle{a,c,b});
        } else {
          const QVector3D a=interpolate(p[positive[0]],p[negative[0]],s[positive[0]],s[negative[0]]);
          const QVector3D b=interpolate(p[positive[0]],p[negative[1]],s[positive[0]],s[negative[1]]);
          const QVector3D c=interpolate(p[positive[1]],p[negative[0]],s[positive[1]],s[negative[0]]);
          const QVector3D d=interpolate(p[positive[1]],p[negative[1]],s[positive[1]],s[negative[1]]);
          triangles.push_back({a,b,c}); triangles.push_back({b,d,c});
        }
      }
    }
  }
  for(auto& triangle:triangles) {
    triangle.ca=colorAt(triangle.a);
    triangle.cb=colorAt(triangle.b);
    triangle.cc=colorAt(triangle.c);
  }
  return triangles;
}
