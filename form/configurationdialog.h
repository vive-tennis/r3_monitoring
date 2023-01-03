#ifndef CONFIGURATIONDIALOG_H
#define CONFIGURATIONDIALOG_H

#include <QDialog>

#include "topic.h"

namespace Ui {
class ConfigurationDialog;
}

class ConfigurationDialog : public QDialog
{
    Q_OBJECT

public:
    enum class TabName {
        TOPICS,
        ACCOUNT,
        CONNECTION,
        ABOUT
    };

public:
    explicit ConfigurationDialog(QWidget *parent = nullptr);
    ~ConfigurationDialog();
    void setTopicList(QList<TopicName> topicList);
    void setTab(TabName tabName);

private slots:
    void on_comboBox_currentIndexChanged(int index);

private:
    Ui::ConfigurationDialog *ui;
    QList<TopicName> _topicList;
    TabName _tabName;
};

#endif // CONFIGURATIONDIALOG_H
