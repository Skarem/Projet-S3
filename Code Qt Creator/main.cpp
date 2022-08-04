#include "mainwindow.h"
#include <QApplication>

bool init = false;
const int delayMs = 50;
int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow w(delayMs);
        w.show();
    return a.exec();
}
