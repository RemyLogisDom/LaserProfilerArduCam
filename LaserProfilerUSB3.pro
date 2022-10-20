
QT += core gui widgets datavisualization
CONFIG += opencv qt warn_on release thread windows largefile

TARGET = LaserProfiler
TEMPLATE = app
OPEN_CV_DIR = C:\opencv-4.6.0\

INCLUDEPATH += $$PWD/Arducam
INCLUDEPATH += $${OPEN_CV_DIR}modules
INCLUDEPATH += C:\opencv-4.6.0\modules\core\include
INCLUDEPATH += C:\opencv-4.6.0\modules\imgcodecs\include
INCLUDEPATH += C:\opencv-4.6.0\modules\highgui\include
INCLUDEPATH += C:\opencv-4.6.0\modules\imgproc\include
INCLUDEPATH += C:\opencv-4.6.0\modules\calib3d\include
INCLUDEPATH += C:\opencv-4.6.0\modules\features2d\include
INCLUDEPATH += C:\opencv-4.6.0\modules\flann\include
INCLUDEPATH += C:\opencv-4.6.0\modules\dnn\include
INCLUDEPATH += C:\opencv-4.6.0\modules\flann\include
INCLUDEPATH += C:\opencv-4.6.0\modules\ml\include
INCLUDEPATH += C:\opencv-4.6.0\modules\objdetect\include
INCLUDEPATH += C:\opencv-4.6.0\modules\photo\include
INCLUDEPATH += C:\opencv-4.6.0\modules\shape\include
INCLUDEPATH += C:\opencv-4.6.0\modules\stitching\include
INCLUDEPATH += C:\opencv-4.6.0\modules\superres\include
INCLUDEPATH += C:\opencv-4.6.0\modules\video\include
INCLUDEPATH += C:\opencv-4.6.0\modules\videoio\include
INCLUDEPATH += C:\opencv-4.6.0\modules\videostab\include
INCLUDEPATH += C:\opencv-4.6.0\include
INCLUDEPATH += C:\opencv-4.6.0-build



LIBS += C:\opencv-4.6.0-build\lib\libopencv_core460.dll.a
LIBS += C:\opencv-4.6.0-build\bin\libopencv_highgui460.dll
LIBS += C:\opencv-4.6.0-build\bin\libopencv_imgcodecs460.dll
LIBS += C:\opencv-4.6.0-build\bin\libopencv_imgproc460.dll
LIBS += C:\opencv-4.6.0-build\bin\libopencv_features2d460.dll
LIBS += C:\opencv-4.6.0-build\bin\libopencv_calib3d460.dll


LIBS += -lArduCamLib -L$$PWD/Arducam
LIBS += -larducam_config_parser -L$$PWD/Arducam
#LIBS += $$PWD/Arducam/ArduCamLib.dll


DEFINES += IB_USE_STD_STRING

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        mainwindow.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
