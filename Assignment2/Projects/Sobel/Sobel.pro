TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt



#INCLUDEPATH += /home/student/Downloads/opencv3.2.0/include ##C:\OpenCV31\build\include ### was c:/opencv31/release/install/include
#INCLUDEPATH += "/usr/local//opencv3/include"  #MACOS X brew installed opencv3
INCLUDEPATH += "/usr/local/include" \
    ../../Sources/OpenCV_samples

 #MACOS X brew installed opencv3

#LIBS += -LC:\opencv31\build\install\x86\mingw\lib
#LIBS += -LC:\opencv31\build\install\x86\mingw\bin
#LIBS += -LC:\opencv31\build\lib
LIBS += -L/usr/local/lib
#LIBS += -L/usr/local/opt/opencv3/lib

## MACOSX & LINUX
LIBS +=    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_imgcodecs \
    -lopencv_features2d \
    -lopencv_calib3d \
    -lopencv_videoio \
#    -lopencv_ffmpeg \
##    -lws2_32 \

SOURCES += main.cpp

HEADERS += \
    ../../Sources/OpenCV_samples/sobel.h
