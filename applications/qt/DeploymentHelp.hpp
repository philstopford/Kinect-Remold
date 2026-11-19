#pragma once
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QClipboard>
#include <QApplication>
#include <QPushButton>

inline QString sensorSetupText() {
#ifdef Q_OS_LINUX
  bool attached=false,denied=false;
  const QDir usb("/sys/bus/usb/devices");
  auto read=[](const QString& path){QFile f(path);return f.open(QIODevice::ReadOnly)?QString::fromLatin1(f.readAll()).trimmed():QString{};};
  for(const QString& name:usb.entryList(QDir::Dirs|QDir::NoDotAndDotDot)) {
    const QString path=usb.filePath(name);
    if(read(path+"/idVendor")!="045e")continue;
    const QString product=read(path+"/idProduct");
    if(product!="02c4"&&product!="02d8")continue;
    attached=true;
    const QString node=QString("/dev/bus/usb/%1/%2").arg(read(path+"/busnum").toInt(),3,10,QChar('0')).arg(read(path+"/devnum").toInt(),3,10,QChar('0'));
    QFile device(node);if(!device.open(QIODevice::ReadWrite))denied=true;
  }
  QString text=denied?"A Kinect v2 is connected, but this user cannot open its USB device.\n\n":
      attached?"A Kinect v2 is connected. If capture is unavailable, check USB 3 speed and close other Kinect applications.\n\n":
      "No Kinect v2 is visible to Linux. Connect its power supply and a USB 3 data cable, then press Refresh.\n\n";
  text+="For first-time USB access, copy the commands below into a terminal. Administrator authentication is required once. Reconnect USB afterward. Desktop access is granted to the active local user; SSH/headless sessions require a separate access policy.\n\n";
  QString rule=QCoreApplication::applicationDirPath()+"/70-kinect-one-remold.rules";
  rule.replace("'","'\\''");
  text+="sudo install -m 0644 '"+rule+"' /etc/udev/rules.d/70-kinect-one-remold.rules\nsudo udevadm control --reload-rules\n\n";
  text+="If access still fails, confirm that you are in an active desktop session and that no other application owns the sensor. Xbox 360 sensors require the separate installed runtime.";
  return text;
#elif defined(Q_OS_WIN)
  return "Connect external power and a USB 3 data cable. Kinect v2 requires the Remold WinUSB driver and runtime service. Run drivers/kinect-one-remold/SETUP.cmd from the extracted archive once, approve the Windows administrator prompt, then press Refresh. This changes the sensor's driver binding; applications using Microsoft's Kinect SDK may require their original driver to be restored.\n\nThe Qt application itself runs without administrator privileges. Xbox 360 sensors require the separate installed runtime.";
#else
  return "Connect the Kinect power supply and a USB 3 data cable. Close other Kinect applications, reconnect USB, then press Refresh. The app includes its native USB runtime. macOS may ask you to allow a newly connected USB accessory.";
#endif
}

inline void showSensorSetup(QWidget* parent) {
  QMessageBox dialog(QMessageBox::Information,"Sensor setup",sensorSetupText(),QMessageBox::Ok,parent);
  dialog.setTextFormat(Qt::PlainText);
  dialog.setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);
  auto* copy=dialog.addButton("Copy instructions",QMessageBox::ActionRole);
  QObject::connect(copy,&QAbstractButton::clicked,&dialog,[]{QApplication::clipboard()->setText(sensorSetupText());});
  dialog.exec();
}
