QT       += core gui openglwidgets charts widgets

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    RangeSlider.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    RangeSlider.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += "D:\PROGRAMS\vcpkg2\installed\x64-windows\include"
INCLUDEPATH += "D:\PROGRAMS\vcpkg2\installed\x64-windows\include\vtk-9.3"
INCLUDEPATH += "D:\PROGRAMS\vcpkg2\installed\x64-windows\include\opencv2"

LIBS += -L"D:/PROGRAMS/vcpkg2/installed/x64-windows/lib"
# VTK Libraries
LIBS += -lvtkCommonCore -lvtkCommonDataModel -lvtkCommonMath -lvtkFiltersCore -lvtkRenderingCore -lvtkRenderingOpenGL2 -lvtkGUISupportQt -lvtksys
# PCL Libraries
LIBS += -lpcl_common -lpcl_io -lpcl_filters -lpcl_visualization -lpcl_search -lpcl_kdtree
# OpenCV Libraries
LIBS += -lopencv_core4 -lopencv_imgproc4 -lopencv_highgui4 -lopencv_imgcodecs4 -lopencv_videoio4 -lopencv_calib3d4 -lopencv_features2d4 -lopencv_flann4 -lopencv_ml4

INCLUDEPATH += D:/PROGRAMS/vcpkg2/installed/x64-windows/include/qt6
LIBS += -L"D:/PROGRAMS/vcpkg2/installed/x64-windows/lib" -lQt6Core -lQt6Gui -lQt6Widgets
