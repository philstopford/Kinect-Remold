#pragma once

#include <QCoreApplication>
#include <QFileInfo>
#include <QFile>
#include <QDir>
#include <QLocalSocket>
#include <QProcess>
#include <QProcessEnvironment>
#include <QTemporaryDir>
#include <QThread>

// Own only the helper started by this GUI. Installed/explicit runtimes win.
class PortableLinuxRuntime {
 public:
  ~PortableLinuxRuntime() {
    if(process_.state()!=QProcess::NotRunning) {
      process_.terminate();
      if(!process_.waitForFinished(5000)){process_.kill();process_.waitForFinished(1000);}
    }
  }
  void start() {
#ifdef Q_OS_LINUX
    if(!qEnvironmentVariableIsEmpty("REMOLD_LINUX_RUNTIME_ONE"))return;
    QLocalSocket installed;installed.connectToServer("/run/kinect-one-remold/sdk.sock");
    if(installed.waitForConnected(100))return;
    const QString helper=QCoreApplication::applicationDirPath()+"/kinect-one-remold";
    if(!QFileInfo::exists(helper)||!directory_.isValid())return;
    qputenv("REMOLD_LINUX_RUNTIME_ONE",QFile::encodeName(directory_.path()));
    auto environment=QProcessEnvironment::systemEnvironment();
    environment.insert("REMOLD_RUNTIME_DIR",directory_.path());
    process_.setProcessEnvironment(environment);process_.setProgram(helper);process_.start();
    if(!process_.waitForStarted(2000))return;
    for(int attempt=0;attempt<50&&process_.state()==QProcess::Running;++attempt) {
      if(QFileInfo::exists(directory_.filePath("sdk.sock")))return;
      QThread::msleep(20);
    }
#endif
  }
 private:
  QTemporaryDir directory_{QDir::tempPath()+"/remold-portable-XXXXXX"};
  QProcess process_;
};
