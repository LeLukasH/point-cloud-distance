#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{

    //qputenv("QT_QPA_PLATFORM", "windows:darkmode=1");
    QApplication a(argc, argv);

    MainWindow w;
    w.setWindowState(Qt::WindowMaximized);
    w.show();
    return a.exec();
}
