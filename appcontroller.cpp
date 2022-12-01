#include "appcontroller.h"

#include <QAction>
#include <QMenu>
#include <QMessageBox>
#include <QApplication>
#include <iostream>

AppController::AppController(QObject *parent)
    : QObject{parent}
{
    _confDialog = std::make_unique<ConfigurationDialog>();
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

