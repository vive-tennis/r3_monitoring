#include "appcontroller.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setQuitOnLastWindowClosed(false);
    a.setApplicationName(APP_NAME);
    a.setOrganizationName(APP_ORG);

    auto controller = std::make_unique<AppController>();
    if (!controller->runSystemTray())
        return 0;


    return a.exec();
}
