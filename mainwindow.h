#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <Q3DSurface>
#include <QtDataVisualization/QSurfaceDataProxy>
#include <QtDataVisualization/QHeightMapSurfaceDataProxy>
#include <QtDataVisualization/QSurface3DSeries>
#include <QtDataVisualization/QLogValue3DAxisFormatter>
#include <QtWidgets/QSlider>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QDir>
#include <QMainWindow>
#include "C:\opencv-4.6.0\include\opencv2\opencv.hpp"
#include <thread>
#include <time.h>
#include <iostream>
#include <istream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include "Arducam\ArduCamLib.h"
#include "Arducam\arducam_config_parser.h"


using namespace std;
using namespace cv;
using namespace QtDataVisualization;


class Capture : public QObject
{
    Q_OBJECT
public slots:
    void process();
signals:
    void finished();
};

class Read : public QObject
{
    Q_OBJECT
public slots:
    void process();
signals:
    void newImage();
    void finished();
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QString loadedFile;
    QTimer logTimer;
    QString lastPath;
    QStringList lastPictureList;
    int pictureIndex = -1;
private:
    int imageCount = 0;
    QString wTitle;
    Ui::MainWindow *ui;
    void setImage(QImage &);
    Q3DSurface graph;
    QHeightMapSurfaceDataProxy heightMapProxy;
    QSurface3DSeries heightMapSeries;
    void newImageFile(bool Blur = false);
private slots:
    void on_pushButton_open_clicked();
    void newImage();
    void on_pushButton_Shoot_clicked();
    void writeSensorRegs(ArduCamHandle &cameraHandle,cv::FileNode rp);
    bool camera_initFromFile(std::string filename, ArduCamHandle &cameraHandle, ArduCamCfg &cameraCfg);
    bool camera_initFromFileCfg(std::string filename, ArduCamHandle &cameraHandle, ArduCamCfg &cameraCfg);
    void on_pushButton_Continus_clicked();
    void on_pushButton_SaveFile_clicked();
    void on_pushButton_ReadFile_clicked();
    void on_spinBoxZmin_valueChanged(int);
    void on_spinBoxExposition_valueChanged(int);
    void on_spinBoxGain_valueChanged(int);
    void process(Mat &);
    void on_pushButton_Zero_clicked();
    void on_Process_stateChanged(int);
    void on_checkBoxLog_stateChanged(int);
protected:
    void closeEvent(QCloseEvent *event);
private slots:
    void logNow();
    void on_spinBoxContourOffset_valueChanged(int);
    void on_checkBoxShowSize_stateChanged(int);
    void on_spinBoxBlur_valueChanged(int);
    void on_pushButton_Previous_clicked();
    void on_pushButton_Next_clicked();
};

#endif // MAINWINDOW_H

