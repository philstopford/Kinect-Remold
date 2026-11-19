#pragma once
#include <QByteArrayView>
#include <QtEndian>

namespace remold {
// Inspect complete packets only. A partial newer frame must never suppress the
// last usable RGB frame; depth packets remain in order for RGB-D registration.
inline bool hasQueuedRgbFrame(QByteArrayView bytes) {
  constexpr qsizetype headerBytes=76;
  qsizetype offset=0;
  while(bytes.size()-offset>=headerBytes) {
    const char* header=bytes.data()+offset;
    if(qFromLittleEndian<quint32>(header)!=0x46534d52u||
       qFromLittleEndian<quint32>(header+4)!=1)return false;
    const quint32 size=qFromLittleEndian<quint32>(header+24);
    if(size>16u*1024u*1024u||bytes.size()-offset-headerBytes<size)return false;
    const quint32 format=qFromLittleEndian<quint32>(header+20);
    if(size&&(format==1005u||format==4u||format==0x41524742u))return true;
    offset+=headerBytes+size;
  }
  return false;
}
}
