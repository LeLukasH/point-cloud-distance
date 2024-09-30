QT       += core gui openglwidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
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

INCLUDEPATH += "D:\PROGRAMS\vcpkg2\installed\x64-windows\include"
INCLUDEPATH += "D:\PROGRAMS\vcpkg2\installed\x64-windows\include\vtk-9.3"

LIBS += -L"D:/PROGRAMS/vcpkg2/installed/x64-windows/lib"
LIBS += -lvtkCommonCore -lvtkCommonDataModel -lvtkCommonMath -lvtkFiltersCore -lvtkRenderingCore -lvtkRenderingOpenGL2 -lvtkGUISupportQt -lvtksys

LIBS += -L"D:/PROGRAMS/vcpkg2/installed/x64-windows/lib"
LIBS += -lpcl_common -lpcl_io -lpcl_filters -lpcl_visualization -lpcl_search
