#include "appcontroller.h"

#include <QAction>
#include <QMenu>
#include <QMessageBox>
#include <QApplication>
#include <iostream>
#include <QSettings>
#include <QFile>
#include <QDir>

AppController::AppController(QObject *parent)
    : QObject{parent}
{
    loadSettings();
    _confDialog = std::make_unique<ConfigurationDialog>();
    _confDialog->setTopicList(_topicList);
}

bool AppController::runSystemTray()
{
    if (QSystemTrayIcon::isSystemTrayAvailable()) {

      // create trayicon
      _systemTray = std::make_unique<QSystemTrayIcon>();
      _systemTray->setIcon(QIcon("://trayicon.jpeg"));
      _systemTray->setToolTip("R3 Monitoring Client");
      _systemTray->setVisible(true);
      // connect(_systemTrayPtr.get(), &QSystemTrayIcon::activated, this, &AppController::trayIconActivated);

      // create menu
      auto trayIconMenu = new QMenu;

      // create actions
      auto topicsAction = new QAction(tr("&Topics"), this);
      auto accountAction = new QAction(tr("&Account"), this);
      auto connectionAction = new QAction(tr("&Connection"), this);
      auto aboutAction = new QAction(tr("&About"), this);
      auto quitAction = new QAction(tr("&Quit"), this);

      // add actions to the menu
      trayIconMenu->addAction(topicsAction);
      trayIconMenu->addAction(accountAction);
      trayIconMenu->addAction(connectionAction);
      trayIconMenu->addSeparator();
      trayIconMenu->addAction(aboutAction);
      trayIconMenu->addAction(quitAction);

      // add the menu to the trayicon
      _systemTray->setContextMenu(trayIconMenu);

      // connect actions to slots
      connect (topicsAction, &QAction::triggered, this, &AppController::on_topicsAction_triggered);
      connect (accountAction, &QAction::triggered, this, &AppController::on_accountAction_triggered);
      connect (connectionAction, &QAction::triggered, this, &AppController::on_connectiontAction_triggered);
      connect (aboutAction, &QAction::triggered, this, &AppController::on_aboutAction_triggered);
      connect (quitAction, &QAction::triggered, this, &AppController::on_quitAction_triggered);
      return true;
    }

    std::cerr << "SystemTrayIcon is not available" << std::endl;
    return false;
}

bool AppController::loadSettings()
{

    QSettings settings(APP_ORG, APP_NAME);
    _serviceName = settings.value("service_name", "r3-monitoring-client").toString();
    _topicsFile = settings.value("topics_file", QDir::homePath() + "/.r3-monitoring/topics.csv").toString();
    _r3monitoringConfFile = settings.value("r3_monitoring_conf_file", QDir::homePath() + "/.r3-monitoring/config.yaml").toString();

    settings.setValue("service_name", _serviceName);
    settings.setValue("topics_file", _topicsFile);
    settings.setValue("r3_monitoring_conf_file", _r3monitoringConfFile);


    // load topics
    QFile file(_topicsFile);
    if (!file.open(QIODevice::ReadOnly)) {
        std::cerr << file.errorString().toStdString() << std::endl;
        return false;
    }

    while (!file.atEnd()) {
        auto line = QString(file.readLine());
        line = line.remove('\n');
        auto splittedLine = line.split(',');
        const int desired_size = 4;
        if (splittedLine.size() != desired_size) {
            std::cerr << "format of " << _topicsFile.toStdString() << " is not correct.  It should be " << desired_size << " columns!" << std::endl;
            return false;
        }

        TopicName topic;
        topic.name = splittedLine[0];
        topic.type = splittedLine[1];
        topic.time_created = splittedLine[2];
        topic.included = QVariant(splittedLine[3]).toBool();

        _topicList.append(topic);
    }

    return true;
}

void AppController::on_topicsAction_triggered()
{
    _confDialog->setTab(ConfigurationDialog::TabName::TOPICS);
    _confDialog->show();
}

void AppController::on_accountAction_triggered()
{
    _confDialog->setTab(ConfigurationDialog::TabName::ACCOUNT);
    _confDialog->show();
}

void AppController::on_connectiontAction_triggered()
{
    _confDialog->setTab(ConfigurationDialog::TabName::CONNECTION);
    _confDialog->show();
}

void AppController::on_aboutAction_triggered()
{
    _confDialog->setTab(ConfigurationDialog::TabName::ABOUT);
    _confDialog->show();
}

void AppController::on_quitAction_triggered()
{
    QApplication::quit();
}

