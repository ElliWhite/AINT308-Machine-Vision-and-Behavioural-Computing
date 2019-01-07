TTEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

# OSX The following lines tells Qmake to use pkg-config for opencv
#QT_CONFIG -= no-pkg-config
#CONFIG  += link_pkgconfig
#PKGCONFIG += /usr/local/Cellar/opencv3/3.2.0/lib/pkgconfig/opencv.pc


## MACOSX & LINUX
#INCLUDEPATH += "/usr/local/include"  #MACOS X brew installed opencv3

#LIBS += -L/usr/local/lib
#LIBS += -L/usr/local/opt/opencv3/lib
#LIBS +=    -lopencv_core \
#    -lopencv_highgui \
#    -lopencv_imgproc \
#    -lopencv_imgcodecs \
#    -lopencv_features2d \
#    -lopencv_calib3d \
#    -lopencv_videoio \
#    -lopencv_ffmpeg \
##    -lws2_32 \

## WIN32
INCLUDEPATH += C:/OpenCV31/build/include ## CHECK LOCATION

LIBS += -LC:\opencv31\build\install\x86\mingw\lib ## CHECK LOCATION
LIBS += -LC:\opencv31\build\install\x86\mingw\bin

LIBS += -LC:/opencv31/build/lib
LIBS +=    -lopencv_core310 \
    -lopencv_highgui310 \
    -lopencv_imgproc310 \
    -lopencv_imgcodecs310 \
    -lopencv_features2d310 \
    -lopencv_calib3d310 \
    -lopencv_videoio310 \
#    -lopencv_ffmpeg310 \
    -lws2_32 \

SOURCES += \
    ../../Sources/Opencv/stereo_match.cpp
