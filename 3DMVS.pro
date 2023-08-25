QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
QMAKE_LFLAGS += -fopenmp
QMAKE_CXXFLAGS+= -fopenmp
# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    dialog.cpp

HEADERS += \
    dialog.h

FORMS += \
    dialog.ui


INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /usr/local/include/vtk-8.2
LIBS += /usr/local/lib/libvtk*.so

INCLUDEPATH += /usr/include/boost
LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so

INCLUDEPATH += /usr/include/pcl-1.13
LIBS += /usr/lib/libpcl_*.so


INCLUDEPATH    += lib/MVG/include/openMVG\
                lib/MVG/include/openMVG_dependencies\
                lib/MVG/include\
                lib/MVG/include/openMVG_dependencies/cereal/include\
                lib/MVG/include/openMVG/third_party/lemon\
                lib/MVG/include/openMVG/third_party\
                lib/MVG/include/openMVG/sfm

LIBS +=  ../lib/MVG/lib/lib*.a\
         ../lib/MVG/lib/libopenMVG_*.a
LIBS +=  /usr/lib/x86_64-linux-gnu/libjpeg.so

LIBS +=  /usr/lib/x86_64-linux-gnu/libpng.so

LIBS +=  /usr/lib/x86_64-linux-gnu/libtiff.so

INCLUDEPATH    += lib/MVG/include/openMVG/third_party/eigen

INCLUDEPATH    += lib/third_party/easyexif

LIBS +=  ../lib/dependencies/libopenMVG_easyexif.a


INCLUDEPATH += /usr/local/include/ceres \
                /usr/local/include/ceres/internal
LIBS += /usr/local/lib/libceres.a



INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2
LIBS += /usr/local/lib/libopencv_*.so \


INCLUDEPATH += /usr/include/suitesparse
LIBS += /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so*
LIBS += /usr/lib/x86_64-linux-gnu/libcholmod.so

INCLUDEPATH += /usr/include/suitesparse/cxsparse
LIBS += /usr/lib/x86_64-linux-gnu/libcxsparse.so*

INCLUDEPATH += /usr/include/gflags
LIBS += /usr/lib/x86_64-linux-gnu/libgflags.so*

INCLUDEPATH += /usr/include/glog
LIBS += /usr/lib/x86_64-linux-gnu/libglog.so*

INCLUDEPATH += /usr/include/x86_64-linux-gnu/openblas-pthread
LIBS += /usr/lib/x86_64-linux-gnu/libblas.so*
LIBS += /usr/lib/x86_64-linux-gnu/liblapack.so*



# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
