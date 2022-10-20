#include <QtDebug>
#include <QCloseEvent>
#include <QtDataVisualization/Q3DSurface>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtGui/QPainter>
#include <QtGui/QScreen>
#include <QtWidgets/QFileDialog>
#ifdef linux
#include <unistd.h>
#include <termios.h>
#endif
#include "mainwindow.h"
#include "ui_mainwindow.h"



ArduCamCfg cameraCfg;
ArduCamHandle cameraHandle;
int color_mode = 0;
bool captureRunning = false;
bool readRunning = false;
bool shoot = false;
Mat image;
Mat image_Original;
QImage saveImage;
int prevWidth = 300;
bool busy = false;
// taille capteur w = 6,119 mm  h = 4,589mm
float Capt_Wmm = 6.119;
float Capt_Hmm = 4.598;
float Capt_WPix = 3664;
float Capt_HPix = 2748;
int beamX = 0;
int beamY = 0;
int beamXOffset = -1;
int beamYOffset = -1;
float beamSizeX = -1;
float beamSizeY = -1;
QString cam_Config = "cam_config";



#define RGB565_RED      0xf800
#define RGB565_GREEN    0x07e0
#define RGB565_BLUE     0x001f

#define str_OPEN "Acquire"
#define str_CLOSE "Stop"


QString type2str(int type)
{
  QString r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth )
  {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

inline QImage cvMatToQImage( const cv::Mat &inMat )
{
   switch ( inMat.type() )
   {
       // 16-bit, 1 channel
        case CV_16UC1:
        {
            QImage image( inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_Grayscale16);
            return image;
        }
        // 8-bit, 4 channel
        case CV_8UC4:
        {
         QImage image( inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_ARGB32 );
         return image;
        }

      // 8-bit, 3 channel
      case CV_8UC3:
      {
         QImage image( inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_RGB888 );
         return image.rgbSwapped();
      }

      // 8-bit, 1 channel
      case CV_8UC1:
      {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
         QImage image( inMat.data,
                       inMat.cols, inMat.rows,
                       static_cast<int>(inMat.step),
                       QImage::Format_Grayscale8);
#else
         static QVector<QRgb>  sColorTable;

         // only create our color table the first time
         if ( sColorTable.isEmpty() )
         {
            sColorTable.resize( 256 );

            for ( int i = 0; i < 256; ++i )
            {
               sColorTable[i] = qRgb( i, i, i );
            }
         }

         QImage image( inMat.data,
                       inMat.cols, inMat.rows,
                       static_cast<int>(inMat.step),
                       QImage::Format_Indexed8 );

         image.setColorTable( sColorTable );
#endif

         return image;
      }

      default:
         qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
         break;
   }

   return QImage();
}

Mat RGB565toMat(Uint8* bytes, int width, int height) {
    unsigned char* temp_data, *ptdata, *data, *data_end;

    data = bytes;
    data_end = bytes + (width * height * 2);

    temp_data = (unsigned char*)malloc(cameraCfg.u32Width * cameraCfg.u32Height * 3);
    ptdata = temp_data;

    Uint8 r, g, b;
    while (data < data_end) {
        unsigned short temp;

        temp = (*data << 8) | *(data + 1);
        r = (temp & RGB565_RED	)	>> 8;
        g = (temp & RGB565_GREEN)	>> 3;
        b = (temp & RGB565_BLUE	)	<< 3;

        switch(color_mode){
        case 1:
            *ptdata++ = r;
            *ptdata++ = g;
            *ptdata++ = b;
            break;
        case 0:
        default:
            *ptdata++ = b;
            *ptdata++ = g;
            *ptdata++ = r;
            break;
        }
        data += 2;
    }

    Mat image = Mat(height, width, CV_8UC3);
    memcpy(image.data, temp_data, cameraCfg.u32Height * cameraCfg.u32Width * 3);
    cv::flip(image, image, 0);
    free(temp_data);
    return image;
}



Mat separationImage(Uint8* bytes, int width, int height)
{
    int width_d = width << 1;
    unsigned char* temp1,*temp2;
    temp1 = (unsigned char*)malloc(width);
    temp2 = (unsigned char*)malloc(width);

    for(int k = 0 ; k < height ;k++){
        for(int i = 0 ,j =0; i < width_d ;i+=2){
            temp1[j] = bytes[i + (k * width_d)];
            temp2[j++] = bytes[i + 1 + (k * width_d)];
        }
        memcpy(bytes + (k * width_d),temp1,width);
        memcpy(bytes + (k * width_d + width),temp2,width);
    }
    cv::Mat image = cv::Mat( height, width_d,CV_8UC1,bytes);
    free(temp1);
    free(temp2);
    return image;
}




Mat YUV422toMat(Uint8* bytes, int width, int height)
{
    cv::Mat image = cv::Mat(height, width, CV_8UC2, bytes);
    cv::cvtColor(image,image,cv::COLOR_YUV2BGR_YUYV);
    return image;
}



Mat dBytesToMat(Uint8* bytes,int bit_width,int width,int height){
    Mat image = Mat(height, width, CV_16UC1, bytes);
    return image;
    //qDebug() << QString("frameData->stImagePara.u8PixelBits = %1").arg(bit_width);
/*    unsigned char* temp_data = (unsigned char*)malloc(width * height);
    int index = 0;
    for(int i = 0 ; i < width * height * 2 ;i+=2){
        unsigned char temp = ((bytes[i + 1] << 8 | bytes[i]) >> (bit_width - 8)) & 0xFF;
        temp_data[index++] = temp;
    }
    Mat image = Mat(height, width, CV_16UC1);
    memcpy(image.data, temp_data, cameraCfg.u32Height * cameraCfg.u32Width);
    free(temp_data);
    return image;*/

/*    unsigned char* temp_data = (unsigned char*)malloc(width * height);
    int index = 0;
    for(int i = 0 ; i < width * height * 2 ;i+=2){
        unsigned char temp = ((bytes[i + 1] << 8 | bytes[i]) >> (bit_width - 8)) & 0xFF;
        temp_data[index++] = temp;
    }
    Mat image = Mat(height, width, CV_8UC1);
    memcpy(image.data, temp_data, cameraCfg.u32Height * cameraCfg.u32Width);
    free(temp_data);
    return image;*/
}



Mat BytestoMat(Uint8* bytes, int width, int height)
{
    Mat image = Mat(height, width, CV_8UC1, bytes);
    return image;
}



Mat ConvertImage(ArduCamOutData* frameData){
    Mat rawImage ;
    Uint8* data = frameData->pu8ImageData;
    int height,width;
    width = cameraCfg.u32Width;
    height = cameraCfg.u32Height;
    switch(cameraCfg.emImageFmtMode){
    case FORMAT_MODE_RGB:
        rawImage = RGB565toMat(data,width,height);
        break;
    case FORMAT_MODE_RAW_D:
        rawImage = separationImage(data,width,height);
        switch(color_mode){
        case RAW_RG:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
            break;
        case RAW_GR:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGR2BGR);
            break;
        case RAW_GB:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGB2BGR);
            break;
        case RAW_BG:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerBG2BGR);
            break;
        default:
            cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
            break;
        }
        break;
    case FORMAT_MODE_MON_D:
        rawImage = separationImage(data,width,height);
        break;
    case FORMAT_MODE_JPG:
        //rawImage = JPGToMat(data,frameData->stImagePara.u32Size);
        break;
    case FORMAT_MODE_RAW:
        if(cameraCfg.u8PixelBytes == 2){
            rawImage = dBytesToMat(data,frameData->stImagePara.u8PixelBits,width,height);
        }else{ //qDebug() << "RAW2";
            rawImage = BytestoMat(data, width, height);
        }
        switch(color_mode){
        case RAW_RG:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
            break;
        case RAW_GR:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGR2BGR);
            break;
        case RAW_GB:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGB2BGR);
            break;
        case RAW_BG:cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerBG2BGR);
            break;
        default:
            cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
            break;
        }
        break;
    case FORMAT_MODE_YUV:
        rawImage = YUV422toMat(data,width,height);
        break;
    case FORMAT_MODE_MON:
        if(cameraCfg.u8PixelBytes == 2){
            rawImage = dBytesToMat(data,frameData->stImagePara.u8PixelBits,width,height);
        }else{
            rawImage = BytestoMat(data, width, height);
        }
        break;
    default:
        if(cameraCfg.u8PixelBytes == 2){
            rawImage = dBytesToMat(data,frameData->stImagePara.u8PixelBits,width,height);
        }else{
            rawImage = BytestoMat(data, width, height);
        }
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2RGB);
        break;
    }
    return rawImage;
}




QString saveformat(const QString &name, const QString &value)
{
    QString hex = value.toUtf8().toHex().toUpper();
        return name + " = (HEX:" + hex + ")\n";
}



QString getvalue(QString search, QString &str)
{
    int l = str.length();
    if (l == 0) return "";

    int i = str.indexOf(search);
    if (i == -1) return "";

    int coma = str.indexOf("(", i);
    if (coma == -1) return "";

    int nextcoma = str.indexOf(")", coma + 1);
    if (nextcoma == -1) nextcoma = str.indexOf("(", coma + 1);
    if (nextcoma == -1) return "";

    QString result = str.mid(coma + 1, nextcoma - coma - 1);
    if (!result.isEmpty())
    {
        if (result.mid(0, 4) == "HEX:")
        {
            QByteArray Hex;
            Hex.append(result.remove(0, 4));
            QByteArray F = QByteArray::fromHex(Hex);
            result = F;
        }
    }
    return result;
}


void Capture::process()
{
    Uint32 rtn_val = ArduCam_beginCaptureImage(cameraHandle);
    if (rtn_val == USB_CAMERA_USB_TASK_ERROR)
    {
        qDebug() << "Error beginning capture";
        captureRunning = false;
        readRunning = false;
    }
    else
    {
        qDebug() << "Capture began";
    }
    while (captureRunning)
    {
        rtn_val = ArduCam_captureImage(cameraHandle);
        /*if (rtn_val == USB_CAMERA_USB_TASK_ERROR)
        {
            qDebug() << "Error capture image";
            captureRunning = false;
        }
        else if(rtn_val > 0xFF)
        {
            qDebug() << "Error capture image";
            captureRunning = false;
        }*/
        //QCoreApplication::processEvents();
    }
    ArduCam_endCaptureImage(cameraHandle);
    qDebug() << "Capture thread stopped";
    ArduCam_close(cameraHandle);
    qDebug() << "Camera closed";
    cameraHandle = 0;
    emit finished();
}



void Read::process()
{
    qDebug() << "Read start";
    ArduCamOutData *frame;
    while (readRunning || shoot)
    {
        if(ArduCam_availableImage(cameraHandle))
        {
            if (ArduCam_readImage(cameraHandle, frame) == USB_CAMERA_NO_ERROR)
            {
                if (!busy)
                {
                    image = ConvertImage(frame);
                    emit(newImage());
                    busy = true;
                    //QCoreApplication::processEvents();
                }
                ArduCam_del(cameraHandle);
                //qDebug() << "New Image";
            }
        }
        shoot = false;
    }
    qDebug() << "Read closed";
    ArduCam_del(cameraHandle);
    emit finished();
}



MainWindow::MainWindow(QWidget *parent) :  QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    wTitle = windowTitle();
    ui->pushButton_Shoot->hide();
    ui->pushButton_Continus->hide();
    ui->pushButton_Previous->hide();
    ui->pushButton_Next->hide();
    ui->pushButton_SaveFile->hide();
    on_pushButton_open_clicked();
    ui->spinBoxExposition->setEnabled(false);
    ui->spinBoxGain->setEnabled(false);
    ui->pushButton_Zero->setEnabled(false);
    ui->lineEditID->setEnabled(false);
    ui->checkBoxLog->setEnabled(false);
    QValue3DAxis *zLog = new QValue3DAxis;
    QLogValue3DAxisFormatter *log = new QLogValue3DAxisFormatter;
    zLog->setFormatter(log);
    graph.setAxisX(new QValue3DAxis);
    graph.setAxisZ(new QValue3DAxis);
    graph.setAxisY(zLog);
    graph.axisX()->setLabelFormat("%.1f X");
    graph.axisY()->setLabelFormat("%.1f Pw");
    graph.axisZ()->setLabelFormat("%.1f Y");
    graph.axisX()->setTitle(QStringLiteral("X"));
    graph.axisY()->setTitle(QStringLiteral("Power"));
    graph.axisZ()->setTitle(QStringLiteral("Y"));
    graph.axisY()->setRange(10, 255);
    //graph.axisY()->setRange(10, 65535);

    QWidget *container = QWidget::createWindowContainer(&graph);
    if (!graph.hasContext())
    {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
    }
    QSize screenSize = graph.screen()->size();
    container->setMinimumSize(QSize(screenSize.width() / 2, screenSize.height() / 2));
    container->setMaximumSize(screenSize);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container->setFocusPolicy(Qt::StrongFocus);
    ui->gridLayout->addWidget(container, 0, 1, 1, 1);

    QFile file("lbm.cfg");
    QTextStream in(&file);
    QString config;
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        config = in.readAll();
        file.close();
    }
    bool ok;
    int x = getvalue("Pos_X", config).toInt(&ok);
    int y = getvalue("Pos_Y", config).toInt(&ok);
    if (ok) move(x, y);
    int w = getvalue("Size_W", config).toInt(&ok);
    int h = getvalue("Size_H", config).toInt(&ok);
    if (ok) resize(w, h);
    int z = getvalue("ZMin", config).toInt(&ok);
    if (ok) ui->spinBoxZmin->setValue(z);
    int e = getvalue("Expo", config).toInt(&ok);
    if (ok) ui->spinBoxExposition->setValue(e);
    int g = getvalue("Gain", config).toInt(&ok);
    if (ok) ui->spinBoxGain->setValue(g);
    int b = getvalue("Blur", config).toInt(&ok);
    if (ok) ui->spinBoxBlur->setValue(b);
    int c = getvalue("Contour", config).toInt(&ok);
    if (ok) ui->spinBoxContourOffset->setValue(c);
    lastPath = getvalue("lastPath", config);
    connect(&logTimer, SIGNAL(timeout()), this, SLOT(logNow()));
    QDir Dir(cam_Config);
    Dir.setFilter(QDir::Files);
    QStringList filters;
    filters << "*.yml" << "*.cfg";
    Dir.setNameFilters(filters);
    QStringList fileList;
    fileList = Dir.entryList();
    ui->listWidgetConfigFile->clear();
    for (int n=0; n<fileList.count(); n++)
    {
        QListWidgetItem *item = new QListWidgetItem;
        QString name = fileList.at(n);
        item->setText(name.remove(".yml").remove(".cfg"));
        ui->listWidgetConfigFile->addItem(item);
    }
    QString profile = getvalue("Profile", config);
    QList <QListWidgetItem*> listProfile;
    listProfile = ui->listWidgetConfigFile->findItems(profile, Qt::MatchExactly);
    if (listProfile.count() > 0)
        ui->listWidgetConfigFile->setCurrentItem(listProfile.first());
    else if (ui->listWidgetConfigFile->count() > 0) ui->listWidgetConfigFile->setCurrentItem(ui->listWidgetConfigFile->item(0));
}



void MainWindow::setImage(QImage &m)
{
    saveImage = m;
    int maxSize = 1900;
    if (m.width() > maxSize) m = m.scaledToWidth(maxSize);
    if (graph.seriesList().count())
    {
        heightMapProxy.setHeightMap(m);
        //heightMapProxy.setHeightMap(m.convertToFormat(QImage::Format_Grayscale16));
    }
    else
    {
        heightMapSeries.setDataProxy(&heightMapProxy);
        graph.addSeries(&heightMapSeries);
        heightMapSeries.setDrawMode(QSurface3DSeries::DrawSurface);
        //heightMapSeries.setFlatShadingEnabled(false);
        QLinearGradient gr;
        gr.setColorAt(0.0, Qt::gray);
        gr.setColorAt(0.1, Qt::darkBlue);
        gr.setColorAt(0.5, QColor(255, 165, 0));
        gr.setColorAt(0.8, Qt::red);
        gr.setColorAt(1.0, Qt::darkRed);
        heightMapSeries.setBaseGradient(gr);
        heightMapSeries.setColorStyle(Q3DTheme::ColorStyleRangeGradient);
        heightMapProxy.setHeightMap(m);
        //heightMapProxy.setHeightMap(m.convertToFormat(QImage::Format_Grayscale16));
    }
}





void MainWindow::on_pushButton_open_clicked()
{
    setWindowTitle(wTitle);
    imageCount = 0;
    loadedFile.clear();
    QString current_sta = ui->pushButton_open->text();
    if(current_sta.compare(str_OPEN) == 0)
    {
        loadedFile.clear();
        ArduCamIndexinfo pUsbIdxArray[16];
        uint camera_num = 0;
        camera_num = ArduCam_scan(pUsbIdxArray);
        if (camera_num == 0)
        {
            QMessageBox msgBox;
            msgBox.setText("No camera found");
            msgBox.exec();
            return;
        }
        //qDebug() << QString("Found %1 camera").arg(camera_num);
        QListWidgetItem *item = ui->listWidgetConfigFile->currentItem();
        if (!item)
        {
            QMessageBox msgBox;
            msgBox.setText("Please \nSelect resolution first");
            msgBox.exec();
            return;
        }
        bool init = false;
        QString configFileName = item->text() + ".yml";
        QString configFileNameFullPath = cam_Config + QDir::separator() + item->text() + ".yml";

        if (QFile::exists(cam_Config + QDir::separator() + configFileName)) init = camera_initFromFile(configFileNameFullPath.toStdString(), cameraHandle, cameraCfg);
        else {
            configFileName = item->text() + ".cfg";
            configFileNameFullPath = cam_Config + QDir::separator() + item->text() + ".cfg";
            if (QFile::exists(cam_Config + QDir::separator() + configFileName)) { init = camera_initFromFileCfg(configFileNameFullPath.toStdString(), cameraHandle, cameraCfg); }
            else {
                QMessageBox msgBox;
                msgBox.setText("Cannot find configuration file " + configFileName);
                msgBox.exec();
                return; }
        }
        if (init)
        {
            if (!cameraHandle) { qDebug() << "No camera handle"; return; }
            //qDebug() << "Camera initialisation";
            if (cameraHandle) ArduCam_setMode(cameraHandle, CONTINUOUS_MODE);
            ui->spinBoxExposition->setEnabled(true);
            ui->spinBoxGain->setEnabled(true);
            ui->pushButton_Zero->setEnabled(true);
            ui->lineEditID->setEnabled(true);
            ui->checkBoxLog->setEnabled(true);
            Uint32 gain;
            Uint32 exposition;
// Read Exposition
            if (cameraHandle) ArduCam_readSensorReg(cameraHandle, 0x3012,  &exposition);
            if (exposition > 100) exposition = ui->spinBoxExposition->value();
            if (cameraHandle) ArduCam_writeSensorReg(cameraHandle, 0x3012,  exposition);
// Read Gain
            if (cameraHandle) ArduCam_readSensorReg(cameraHandle, 0x3028,  &gain);
            if (gain > 100) gain = ui->spinBoxGain->value();
            if (cameraHandle) ArduCam_writeSensorReg(cameraHandle, 0x3028,  gain);
            ui->pushButton_open->setText(str_CLOSE);
            ui->pushButton_ReadFile->hide();
            ui->pushButton_Previous->hide();
            ui->pushButton_Next->hide();
            ui->pushButton_Shoot->show();
            ui->pushButton_Continus->show();
            ui->listWidgetConfigFile->setEnabled(false);
            ui->pushButton_Continus->setEnabled(false);
            ui->pushButton_Continus->setEnabled(true);
            captureRunning = true;
            QThread *threadC = new QThread;
            Capture *capture = new Capture;
            capture->moveToThread(threadC);
            connect(threadC, SIGNAL(started()), capture, SLOT(process()));
            connect(capture, SIGNAL(finished()), threadC, SLOT(quit()));
            connect(capture, SIGNAL(finished()), capture, SLOT(deleteLater()));
            connect(threadC, SIGNAL(finished()), threadC, SLOT(deleteLater()));
            threadC->start();

            readRunning = true;
            QThread *threadR = new QThread;
            Read *read = new Read;
            read->moveToThread(threadR);
            connect(threadR, SIGNAL(started()), read, SLOT(process()));
            connect(read, SIGNAL(finished()), threadR, SLOT(quit()));
            connect(read, SIGNAL(finished()), read, SLOT(deleteLater()));
            connect(threadR, SIGNAL(finished()), threadR, SLOT(deleteLater()));
            connect(read, SIGNAL(newImage()), this, SLOT(newImage()), Qt::QueuedConnection);
            threadR->start();
        }
    }
    else
    {
        ui->pushButton_open->setText(str_OPEN);
        ui->pushButton_Continus->setEnabled(false);
        //ui->pushButton_SaveFile->setEnabled(false);
        captureRunning = false;
        readRunning = false;
        ui->spinBoxExposition->setEnabled(false);
        ui->spinBoxGain->setEnabled(false);
        ui->pushButton_Zero->setEnabled(false);
        ui->lineEditID->setEnabled(false);
        ui->checkBoxLog->setEnabled(false);
        if (cameraHandle) ArduCam_del(cameraHandle);
        ui->pushButton_ReadFile->show();
        ui->pushButton_SaveFile->hide();
        if (!lastPictureList.isEmpty())
        {
            ui->pushButton_Previous->show();
            ui->pushButton_Next->show();
        }
        ui->pushButton_Shoot->hide();
        ui->pushButton_Continus->hide();
        ui->listWidgetConfigFile->setEnabled(true);
        ui->pushButton_Continus->setEnabled(true);
    }
}


void configBoardCfg(ArduCamHandle &cameraHandle, Config config) {
    uint8_t u8Buf[10];
    for (int n = 0; n < config.params[3]; n++) {
        u8Buf[n] = config.params[4 + n];
    }
    ArduCam_setboardConfig(cameraHandle, config.params[0], config.params[1],
        config.params[2], config.params[3], u8Buf);
}

void configBoard(ArduCamHandle &cameraHandle,cv::FileNode bp)
{
    std::string hexStr;
    for (uint i = 0; i<bp.size(); i++) {
        uint8_t u8Buf[10];
        for (uint j = 0; j < bp[i][4].size(); j++){
            bp[i][4][j] >> hexStr;
            u8Buf[j] = std::stoul(hexStr, nullptr, 16);
        }

        bp[i][0] >> hexStr;
        Uint8 u8Command = std::stoul(hexStr, nullptr, 16);
        bp[i][1] >> hexStr;
        Uint16 u16Value = std::stoul(hexStr, nullptr, 16);
        bp[i][2] >> hexStr;
        Uint16 u16Index = std::stoul(hexStr, nullptr, 16);
        bp[i][3] >> hexStr;
        Uint32 u32BufSize = std::stoul(hexStr, nullptr, 16);
        ArduCam_setboardConfig(cameraHandle, u8Command,u16Value,u16Index, u32BufSize, u8Buf);
    }
}



void MainWindow::writeSensorRegs(ArduCamHandle &cameraHandle,cv::FileNode rp)
{
    std::string hexStr;
    for (uint i = 0; i < rp.size(); i++) {
        rp[i][0] >> hexStr;
        if(hexStr.compare("DELAY") == 0){
            rp[i][1] >> hexStr;
            Uint32 delay_time = std::stoul(hexStr, nullptr, 10);
#ifdef linux
            usleep(1000 * delay_time);
#endif
#ifdef _WIN32
            Sleep(delay_time);
#endif
            continue;
        }
        Uint32 addr = std::stoul(hexStr, nullptr, 16);
        rp[i][1] >> hexStr;
        Uint32 val = std::stoul(hexStr, nullptr, 16);
        ArduCam_writeSensorReg(cameraHandle, addr, val);
    }
}





/**
 * read config file and open the camera.
 * @param filename : path/config_file_name
 * @param cameraHandle : camera handle
 * @param cameraCfg :camera config struct
 * @return TURE or FALSE
 * */



bool MainWindow::camera_initFromFile(std::string filename, ArduCamHandle &cameraHandle, ArduCamCfg &cameraCfg)
{
    FileStorage cfg;
    if (cfg.open(filename, FileStorage::READ))
    {
        cv::FileNode cp = cfg["camera_parameter"];
        int value;
        std::string hexStr;

        cp["I2C_MODE"] >> value;
        switch (value)
        {
            case 0: cameraCfg.emI2cMode = I2C_MODE_8_8; break;
            case 1: cameraCfg.emI2cMode = I2C_MODE_8_16; break;
            case 2: cameraCfg.emI2cMode = I2C_MODE_16_8; break;
            case 3: cameraCfg.emI2cMode = I2C_MODE_16_16; break;
            default: break;
        }

        cp["FORMAT"][0] >> value;
        cp["FORMAT"][1] >> color_mode;
        switch (value)
        {
            case 0: cameraCfg.emImageFmtMode = FORMAT_MODE_RAW; break;
            case 1: cameraCfg.emImageFmtMode = FORMAT_MODE_RGB; break;
            case 2: cameraCfg.emImageFmtMode = FORMAT_MODE_YUV; break;
            case 3: cameraCfg.emImageFmtMode = FORMAT_MODE_JPG; break;
            case 4: cameraCfg.emImageFmtMode = FORMAT_MODE_MON; break;
            case 5: cameraCfg.emImageFmtMode = FORMAT_MODE_RAW_D; break;
            case 6: cameraCfg.emImageFmtMode = FORMAT_MODE_MON_D; break;
            default: break;
        }

        cp["SIZE"][0] >> value; cameraCfg.u32Width = value;
        cp["SIZE"][1] >> value; cameraCfg.u32Height = value;

        cp["I2C_ADDR"] >> hexStr; cameraCfg.u32I2cAddr = std::stoul(hexStr, nullptr, 16);
        cp["BIT_WIDTH"] >> value; cameraCfg.u8PixelBits = value;
        cp["TRANS_LVL"] >> value; cameraCfg.u32TransLvl = value;

        if(cameraCfg.u8PixelBits <= 8){
            cameraCfg.u8PixelBytes = 1;
        }else if(cameraCfg.u8PixelBits > 8 && cameraCfg.u8PixelBits <= 16){
            cameraCfg.u8PixelBytes = 2;
        }

        //int ret_val = ArduCam_open(cameraHandle, &cameraCfg,0);
        int ret_val = ArduCam_autoopen(cameraHandle, &cameraCfg);
        if (ret_val == USB_CAMERA_NO_ERROR)
        {
            //ArduCam_enableForceRead(cameraHandle);	//Force display image
            FileNode board_param = cfg["board_parameter"];
            //cv::FileNode bp = cfg["board_parameter_dev2"];

            ArduCam_writeReg_8_8(cameraHandle, 0x46, 3, 0x00);
            configBoard(cameraHandle,board_param);

            //confirm usb model  (usbType will be assigned after calling the ArduCam_autoopen or ArduCam_open method)
            switch(cameraCfg.usbType){
            case USB_1:
            case USB_2:
                configBoard(cameraHandle,cfg["board_parameter_dev2"]);
                break;
            case USB_3:configBoard(cameraHandle,cfg["board_parameter_dev3_inf3"]);		break;
            case USB_3_2:configBoard(cameraHandle,cfg["board_parameter_dev3_inf2"]);	break;
            }

            writeSensorRegs(cameraHandle,cfg["register_parameter"]);

            switch(cameraCfg.usbType){
            case USB_1:
            case USB_2:break;
            case USB_3:writeSensorRegs(cameraHandle,cfg["register_parameter_dev3_inf3"]);	break;
            case USB_3_2:writeSensorRegs(cameraHandle,cfg["register_parameter_dev3_inf2"]);	break;
            }

            unsigned char u8TmpData[16];
            ArduCam_readUserData(cameraHandle, 0x400-16, 16, u8TmpData );
            printf( "Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c\n",
                  u8TmpData[0], u8TmpData[1], u8TmpData[2], u8TmpData[3],
                  u8TmpData[4], u8TmpData[5], u8TmpData[6], u8TmpData[7],
                  u8TmpData[8], u8TmpData[9], u8TmpData[10], u8TmpData[11] );
        }
        else
        {
            std::cout << "Cannot open camera.rtn_val = "<< ret_val << std::endl;
            cfg.release();
            return false;
        }

        cfg.release();
        return true;
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Cannot find configuration file");
        msgBox.exec();
        return false;
    }
}



bool MainWindow::camera_initFromFileCfg(std::string filename, ArduCamHandle &cameraHandle, ArduCamCfg &cameraCfg)
{
    CameraConfigs cam_cfgs;
    memset(&cam_cfgs, 0x00, sizeof(CameraConfigs));
    if (arducam_parse_config(filename.c_str(), &cam_cfgs)) {
        return false;
    }

    CameraParam *cam_param = &cam_cfgs.camera_param;
    Config *configs = cam_cfgs.configs;
    int configs_length = cam_cfgs.configs_length;

    switch (cam_param->i2c_mode) {
    case 0: cameraCfg.emI2cMode = I2C_MODE_8_8; break;
    case 1: cameraCfg.emI2cMode = I2C_MODE_8_16; break;
    case 2: cameraCfg.emI2cMode = I2C_MODE_16_8; break;
    case 3: cameraCfg.emI2cMode = I2C_MODE_16_16; break;
    default: break;
    }

    color_mode = cam_param->format & 0xFF;
    switch (cam_param->format >> 8) {
    case 0: cameraCfg.emImageFmtMode = FORMAT_MODE_RAW; break;
    case 1: cameraCfg.emImageFmtMode = FORMAT_MODE_RGB; break;
    case 2: cameraCfg.emImageFmtMode = FORMAT_MODE_YUV; break;
    case 3: cameraCfg.emImageFmtMode = FORMAT_MODE_JPG; break;
    case 4: cameraCfg.emImageFmtMode = FORMAT_MODE_MON; break;
    case 5: cameraCfg.emImageFmtMode = FORMAT_MODE_RAW_D; break;
    case 6: cameraCfg.emImageFmtMode = FORMAT_MODE_MON_D; break;
    default: break; }

    cameraCfg.u32Width = cam_param->width;
    cameraCfg.u32Height = cam_param->height;

    cameraCfg.u32I2cAddr = cam_param->i2c_addr;
    cameraCfg.u8PixelBits = cam_param->bit_width;
    cameraCfg.u32TransLvl = cam_param->trans_lvl;

    if (cameraCfg.u8PixelBits <= 8) {
        cameraCfg.u8PixelBytes = 1;
    }
    else if (cameraCfg.u8PixelBits > 8 && cameraCfg.u8PixelBits <= 16) {
        cameraCfg.u8PixelBytes = 2;
        //save_raw = true;
    }
    //int ret_val = ArduCam_open(cameraHandle, &cameraCfg, 0);
    int ret_val = ArduCam_autoopen(cameraHandle, &cameraCfg);
    if (ret_val == USB_CAMERA_NO_ERROR) {
        qDebug() << "Camera open ok";
        //ArduCam_enableForceRead(cameraHandle);	//Force display image
        for (int i = 0; i < configs_length; i++) {
            uint32_t type = configs[i].type;
            if (((type >> 16) & 0xFF) && ((type >> 16) & 0xFF) != cameraCfg.usbType)
                continue;
            switch (type & 0xFFFF) {
            case CONFIG_TYPE_REG:
                ArduCam_writeSensorReg(cameraHandle, configs[i].params[0], configs[i].params[1]);
                break;
            case CONFIG_TYPE_DELAY:
#ifdef linux
                usleep(1000 * configs[i].params[0]);
#endif
#ifdef _WIN32
                Sleep(configs[i].params[0]);
#endif
                break;
            case CONFIG_TYPE_VRCMD:
                configBoardCfg(cameraHandle, configs[i]);
                break;
            }
        }
        unsigned char u8TmpData[16];
        ArduCam_readUserData(cameraHandle, 0x400 - 16, 16, u8TmpData);
        printf("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c\n",
            u8TmpData[0], u8TmpData[1], u8TmpData[2], u8TmpData[3],
            u8TmpData[4], u8TmpData[5], u8TmpData[6], u8TmpData[7],
            u8TmpData[8], u8TmpData[9], u8TmpData[10], u8TmpData[11]);
    }
    else {
        qDebug() << QString("Cannot open camera.rtn_val = %1").arg(ret_val);
        return false;   }
    return true;
}


MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::newImage()
{
    imageCount ++;
    if (loadedFile.isEmpty())
    {
        QString dot;
        if (imageCount == 0) dot = "";
        else if (imageCount == 1) dot = ".";
        else if (imageCount == 2) dot = "..";
        else if (imageCount == 3) dot = "...";
        else if (imageCount == 4) dot = "....";
        else if (imageCount == 5) dot = " ...";
        else if (imageCount == 6) dot = "  ..";
        else if (imageCount == 7) dot = "   .";
        else imageCount = 0;
        setWindowTitle(wTitle + " " + dot);
    }
    prevWidth = ui->ImageDisplay->size().width();
    Mat rM;
    Mat m = image;
    blur(image, m, cv::Size(ui->spinBoxBlur->value(), ui->spinBoxBlur->value()));
    if (ui->Process->isChecked())
    {
        process(m);
    }
    else
    {
        ui->textBrowser->clear();
        ui->labelBeamSize->clear();
        QImage dest = cvMatToQImage(m).convertToFormat(QImage::Format_RGB32);
        setImage(dest);
        if (m.size().width > prevWidth)
            cv::resize(m, rM, cv::Size(prevWidth, m.rows*prevWidth/m.cols), 0, 0, INTER_LINEAR);
        QImage img = cvMatToQImage(rM).convertToFormat(QImage::Format_RGB32);
        ui->ImageDisplay->setPixmap(QPixmap::fromImage(img));
    }
    QCoreApplication::processEvents();
    busy = false;
}




void MainWindow::newImageFile(bool Blur)
{
    imageCount ++;
    prevWidth = ui->ImageDisplay->size().width();
    Mat rM;
    Mat m = image;
    if (Blur) blur(image, m, cv::Size(ui->spinBoxBlur->value(), ui->spinBoxBlur->value()));
    if (ui->Process->isChecked())
    {
        process(m);
    }
    else
    {
        ui->textBrowser->clear();
        ui->labelBeamSize->clear();
        QImage dest = cvMatToQImage(m).convertToFormat(QImage::Format_RGB32);
        setImage(dest);
        if (m.size().width > prevWidth)
            cv::resize(m, rM, cv::Size(prevWidth, m.rows*prevWidth/m.cols), 0, 0, INTER_LINEAR);
        QImage img = cvMatToQImage(rM).convertToFormat(QImage::Format_RGB32);
        ui->ImageDisplay->setPixmap(QPixmap::fromImage(img));
    }
    busy = false;
}


void MainWindow::closeEvent(QCloseEvent *)
{
    captureRunning = false;
    readRunning = false;
    QFile file("lbm.cfg");
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&file);
        QString str;
        str += saveformat("Pos_X", QString("%1").arg(pos().x()));
        str += saveformat("Pos_Y", QString("%1").arg(pos().y()));
        str += saveformat("Size_W", QString("%1").arg(size().width()));
        str += saveformat("Size_H", QString("%1").arg(size().height()));
        str += saveformat("ZMin", QString("%1").arg(ui->spinBoxZmin->value()));
        str += saveformat("Expo", QString("%1").arg(ui->spinBoxExposition->value()));
        str += saveformat("Gain", QString("%1").arg(ui->spinBoxGain->value()));
        str += saveformat("Blur", QString("%1").arg(ui->spinBoxBlur->value()));
        str += saveformat("Contour", QString("%1").arg(ui->spinBoxContourOffset->value()));
        str += saveformat("lastPath", lastPath);
        QListWidgetItem *item = ui->listWidgetConfigFile->currentItem();
        if (item) str += saveformat("Profile", QString("%1").arg(item->text()));
        out << str;
    }
    destroyAllWindows();
    exit(0);
}



void MainWindow::on_pushButton_Shoot_clicked()
{
    loadedFile.clear();
    ui->pushButton_SaveFile->setEnabled(true);
    ui->pushButton_ReadFile->setEnabled(true);
    ui->spinBoxGain->setEnabled(false);
    ui->spinBoxExposition->setEnabled(false);
    ui->pushButton_SaveFile->show();
    readRunning = false;
    shoot = true;
    newImage();
}



void MainWindow::on_pushButton_Continus_clicked()
{
    loadedFile.clear();
    ui->pushButton_SaveFile->setEnabled(false);
    ui->pushButton_ReadFile->setEnabled(false);
    ui->spinBoxGain->setEnabled(true);
    ui->spinBoxExposition->setEnabled(true);
    ui->pushButton_SaveFile->hide();
    readRunning = true;
    shoot = false;
    readRunning = true;
    QThread *threadR = new QThread;
    Read *read = new Read;
    read->moveToThread(threadR);
    connect(threadR, SIGNAL(started()), read, SLOT(process()));
    connect(read, SIGNAL(finished()), threadR, SLOT(quit()));
    connect(read, SIGNAL(finished()), read, SLOT(deleteLater()));
    connect(threadR, SIGNAL(finished()), threadR, SLOT(deleteLater()));
    connect(read, SIGNAL(newImage()), this, SLOT(newImage()), Qt::QueuedConnection);
    threadR->start();
}



void MainWindow::on_pushButton_SaveFile_clicked()
{
    if (lastPath.isEmpty()) lastPath = QDir::currentPath();
    QString selFilter="Bitmap files (*.bmp)";
    QString fileName = QFileDialog::getSaveFileName(this, "Save file", lastPath, "Bitmap files (*.bmp);;All files (*.*)",&selFilter);
    if (!fileName.endsWith("bmp")) fileName = fileName + ".bmp";
    if (fileName.isEmpty()) return;
    //imwrite(fileName.toStdString(), m);
    //QImage dest = cvMatToQImage(m).convertToFormat(QImage::Format_RGB32);
    saveImage.save(fileName);
}



void MainWindow::on_pushButton_Previous_clicked()
{
    if (lastPictureList.isEmpty()) return;
    if (pictureIndex > 0)
    {
        pictureIndex --;
        ui->pushButton_Next->show();
    }
    if (pictureIndex == 0) ui->pushButton_Previous->hide();
    QString filename = lastPictureList.at(pictureIndex);
    loadedFile = filename;
    QFileInfo info(loadedFile);
    setWindowTitle(wTitle + " " + info.fileName());
    QImage img = QImage(filename);
    if (img.format() == QImage::Format_Invalid)
    {
        QMessageBox msgBox;
        msgBox.setText("Invalid format");
        msgBox.exec();
        return;
    }
    loadedFile = filename;
    image = imread(filename.toLocal8Bit().toStdString(), IMREAD_UNCHANGED);
    if (image.type() == CV_8UC3) cvtColor(image, image, COLOR_RGBA2GRAY);
    newImageFile();
}




void MainWindow::on_pushButton_Next_clicked()
{
    if (pictureIndex < (lastPictureList.count()-1))
    {
        pictureIndex ++;
        ui->pushButton_Previous->show();
    }
    if (pictureIndex == (lastPictureList.count()-1)) ui->pushButton_Next->hide();
    QString filename = lastPictureList.at(pictureIndex);
    loadedFile = filename;
    QFileInfo info(loadedFile);
    setWindowTitle(wTitle + " " + info.fileName());
    QImage img = QImage(filename);
    if (img.format() == QImage::Format_Invalid)
    {
        QMessageBox msgBox;
        msgBox.setText("Invalid format");
        msgBox.exec();
        return;
    }
    loadedFile = filename;
    image = imread(filename.toLocal8Bit().toStdString(), IMREAD_UNCHANGED);
    if (image.type() == CV_8UC3) cvtColor(image, image, COLOR_RGBA2GRAY);
    newImageFile();
}


void MainWindow::on_pushButton_ReadFile_clicked()
{
    if (lastPictureList.count() > 0)
        while (pictureIndex < (lastPictureList.count() - 1)) lastPictureList.removeLast();
    ui->pushButton_Next->hide();
    if (lastPath.isEmpty()) lastPath = QDir::currentPath();
    QString filename = QFileDialog::getOpenFileName(this, "Open Bitmap file", lastPath, "All files (*.*) ;; Bitmap files (*.bmp)");
    if (lastPictureList.count() > 0)
        if (filename == lastPictureList.last()) return;
    if(!filename.isEmpty())
    {
        QImage img = QImage(filename);
        if (img.format() == QImage::Format_Invalid)
        {
            QMessageBox msgBox;
            msgBox.setText("Invalid format");
            msgBox.exec();
            return;
        }
        loadedFile = filename;
        QFileInfo info(loadedFile);
        QDir dir = info.absoluteDir();
        setWindowTitle(wTitle + " " + info.fileName());
        lastPictureList.append(loadedFile);
        pictureIndex = lastPictureList.count() - 1;
        lastPath = dir.absolutePath();
        if (lastPictureList.isEmpty()) lastPictureList.append(loadedFile);
        else if (lastPictureList.last() != loadedFile) lastPictureList.append(loadedFile);
        if (lastPictureList.count() > 1) ui->pushButton_Previous->show();
        image = imread(filename.toLocal8Bit().toStdString(), IMREAD_UNCHANGED);
        if (image.type() == CV_8UC3) cvtColor(image, image, COLOR_RGBA2GRAY);
        newImageFile();
    }
}



void MainWindow::process(Mat &m)
{
    bool crop = false;
    cv::Rect rectCrop, checkCrop;
    int thresh = 50;
    RNG rng(12345);
    Mat gray;
    Mat threshold_output;
    vector<vector<Point>> contours;
    GaussianBlur(m, gray, Size(7, 7), 0);
    dilate(gray, gray, 0);
    erode(gray, gray, 0);
    threshold( gray, threshold_output, thresh, 255, THRESH_BINARY );

    //QMessageBox msgBox;
    //QString ty = "Image type : " + type2str(gray.type());
    //msgBox.setText(ty + QString(" %2 x %3").arg(m.cols).arg(m.rows));
    //msgBox.exec();

    findContours(threshold_output, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<Rect> minRect(contours.size());
    vector<RotatedRect> minEllipse(contours.size());
    QString text, beamSizeStr;
    int area = 0;
    int index = -1;
    for (uint i=0; i<contours.size(); i++)
    {
        if (contours[i].size() > 5)
        {
            minEllipse[i] = fitEllipse(Mat(contours[i]));
            minRect[i] = boundingRect(Mat(contours[i]));
    // beam size calculation
            int a = minEllipse[i].size.width;
            int b = minEllipse[i].size.height;
            float A = Capt_Wmm / m.cols * a;
            float B = Capt_Hmm / m.rows * b;
            int x = a * b;
            if ((x > area) && (contours[i].size() > 5))
            {
                beamSizeX = A;
                beamSizeY = B;
                text = QString("Beam size : %1 x %2 mm").arg(A, 0, 'f', 2).arg(B, 0, 'f', 2);
                beamSizeStr = text;
                beamX = minEllipse[i].center.x;
                beamY = minEllipse[i].center.y;
                if (beamXOffset < 0)
                {
                    text += QString("\nBeam position %1 : %2").arg(beamX).arg(beamY);
                }
                else
                {
                    text += QString("\nBeam position %1 : %2").arg(beamXOffset-beamX).arg(beamYOffset-beamY);
                }
                text += QString("\nCamera image size : %1 x %2 mm").arg(m.size().width).arg(m.size().height);
                rectCrop = minRect[i];
                crop = true;
                area = x;
                index = i;
            }
        }
    }
    ui->textBrowser->setText(text);
    ui->labelBeamSize->setText(beamSizeStr);
    int expand = (5) * ui->spinBoxContourOffset->value();
    Mat cropView;
    if ((crop) && (index != -1))
    {
        // expand rectangle around beam
        int max = 0;
        checkCrop = rectCrop;
        while (((checkCrop & cv::Rect(0, 0, m.cols, m.rows)) == checkCrop) && (max < expand))
        {
            rectCrop = checkCrop;
            cv::Point inflationPoint(-10, -10);
            cv::Size inflationSize(20, 20);
            checkCrop += inflationPoint;
            checkCrop += inflationSize;
            max += 1;
        }
        if (max > 0)
        {
            Mat croppedImage = m(rectCrop);
            //if (croppedImage.cols > 2000) cv::resize(croppedImage, croppedImage, cv::Size(w/2, h/2), 0, 0, CV_INTER_LINEAR);
            ui->textBrowser->append(QString("Processed image size : %1 x %2 mm").arg(croppedImage.size().width).arg(croppedImage.size().height));
            QImage dest = cvMatToQImage(croppedImage).convertToFormat(QImage::Format_RGB32);
            cropView = croppedImage;
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            //drawContours( m, contours, index, color, 1, 8, vector<Vec4i>(), 0, Point() );
            if (ui->checkBoxShowSize->isChecked()) ellipse( m, minEllipse[index], color, 2, 1 );
            //rectangle( m, minRect[index].tl(), minRect[index].br(), color, 2, 2 );
            cropView = m(rectCrop);
            setImage(dest);
            //if (cropView.size().width > prevWidth) cv::resize(cropView, cropView, cv::Size(prevWidth, cropView.rows*prevWidth/cropView.cols), 0, 0, INTER_LINEAR);
            if (cropView.size().width > prevWidth)
                cv::resize(cropView, cropView, cv::Size(prevWidth, cropView.rows*prevWidth/cropView.cols), 0, 0, INTER_LINEAR);
            QImage img = cvMatToQImage(cropView).convertToFormat(QImage::Format_RGB32);
            QImage grImg(11, 256, QImage::Format_RGB32);
            QPainter painter(&grImg);
            QLinearGradient gr(0, 10, 0, 255);
            gr.setColorAt(0.0, Qt::black);
            gr.setColorAt(0.1, Qt::darkBlue);
            gr.setColorAt(0.5, QColor(255, 165, 0));
            gr.setColorAt(0.8, Qt::red);
            gr.setColorAt(1.0, Qt::darkRed);
            painter.fillRect(0, 0, 10, 255, gr);
            for(int w = 0; w <img.width(); ++w) {
                for(int h = 0; h < img.height(); ++h) {
                    int g = qGray(img.pixel(w, h));
                    QPoint p(w, h);
                    QPoint c(0, g);
                    img.setPixel(p, grImg.pixel(c));
                }
            }
            //ui->ImageDisplay->setPixmap(QPixmap::fromImage(grImg));
            ui->ImageDisplay->setPixmap(QPixmap::fromImage(img));
        }
    }
    else
    {
        //if (m.cols > 2000) cv::resize(m, m, cv::Size(w/2, h/2), 0, 0, CV_INTER_LINEAR);
        QImage dest = cvMatToQImage(m).convertToFormat(QImage::Format_RGB32);
        setImage(dest);
    }
}


//graph.axisX()->setRange(xMin, ui->spinBoxXmax->value());
//graph.axisX()->setRange(ui->spinBoxXmin->value(), xMax);
//graph.axisZ()->setRange(yMin, ui->spinBoxYmax->value());
//graph.axisZ()->setRange(ui->spinBoxYmin->value(), yMax);


void MainWindow::on_spinBoxZmin_valueChanged(int zMin)
{
    graph.axisY()->setRange(zMin, 255);
}

void MainWindow::on_spinBoxExposition_valueChanged(int value)
{
    if (cameraHandle) ArduCam_writeSensorReg(cameraHandle, 0x3012,  uint(value));
}

void MainWindow::on_spinBoxGain_valueChanged(int value)
{
    if (cameraHandle) ArduCam_writeSensorReg(cameraHandle, 0x3028,  uint(value));
}




void MainWindow::on_pushButton_Zero_clicked()
{
    if (beamXOffset < 0)
    {
        beamXOffset = beamX;
        beamYOffset = beamY;
        ui->pushButton_Zero->setText("Relative");
    }
    else
    {
        ui->pushButton_Zero->setText("Absolute");
        beamXOffset = -1;
        beamYOffset = -1;
    }
}


void MainWindow::on_spinBoxBlur_valueChanged(int)
{
    on_Process_stateChanged(ui->Process->isChecked());
}



void MainWindow::on_spinBoxContourOffset_valueChanged(int)
{
    on_Process_stateChanged(ui->Process->isChecked());
}



void MainWindow::on_checkBoxShowSize_stateChanged(int)
{
    if (!ui->Process->isChecked()) ui->Process->setChecked(true);
    on_Process_stateChanged(ui->Process->isChecked());
}



void MainWindow::on_Process_stateChanged(int)
{
    if (!loadedFile.isEmpty())
    {
        image = imread(loadedFile.toLocal8Bit().toStdString(), IMREAD_UNCHANGED);
        cvtColor(image, image, COLOR_RGBA2GRAY);
        newImageFile(true);
    }
}



void MainWindow::on_checkBoxLog_stateChanged(int state)
{
    if (state)
    {
        if (ui->lineEditID->text().isEmpty())
        {
            QMessageBox msgBox;
            msgBox.setText("Please do not leave log name box empty");
            msgBox.exec();
            return;
        }
        ui->Process->setCheckState(Qt::Checked);
        ui->lineEditID->setEnabled(false);
        logTimer.start(60000);
    }
    else
    {
        logTimer.stop();
        ui->lineEditID->setEnabled(true);
    }
}




void MainWindow::logNow()
{
    QString fileName = ui->lineEditID->text();
    if (fileName.isEmpty()) return;
    if (!fileName.endsWith(".dat")) fileName.append(".dat");
    QFile file(fileName);
    if (file.open(QIODevice::Append | QIODevice::Text))
    {
        QTextStream out(&file);
        QString str;
        if (beamXOffset < 0)
        {
            str += QString("%1:%2:%3:%4\n").arg(beamX).arg(beamY).arg(beamSizeX, 0, 'f', 2).arg(beamSizeY, 0, 'f', 2);
        }
        else
        {
            str += QString("%1:%2:%3:%4\n").arg(beamXOffset-beamX).arg(beamYOffset-beamY).arg(beamSizeX, 0, 'f', 2).arg(beamSizeY, 0, 'f', 2);
        }
        out << str;
        file.close();
    }
}



