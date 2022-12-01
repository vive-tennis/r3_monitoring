#include "configurationdialog.h"
#include "ui_configurationdialog.h"

ConfigurationDialog::ConfigurationDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConfigurationDialog)
{
    ui->setupUi(this);
}

ConfigurationDialog::~ConfigurationDialog()
{
    delete ui;
}

void ConfigurationDialog::setTab(TabName tabName)
{
    switch (tabName) {
    case TabName::TOPICS:
        ui->tabWidget->setCurrentWidget(ui->TopicsWidget);
        break;
    case TabName::ACCOUNT:
        ui->tabWidget->setCurrentWidget(ui->AccountWidget);
        break;
    case TabName::CONNECTION:
        ui->tabWidget->setCurrentWidget(ui->ConnectionWidget);
        break;
    case TabName::ABOUT:
        ui->tabWidget->setCurrentWidget(ui->AboutWidget);
        break;
    default:
        break;
    }
}
