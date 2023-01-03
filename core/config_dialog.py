from PyQt5 import QtWidgets, uic


class ConfigDialog(QtWidgets.QDialog):
    class TabName:
        TOPICS = 0
        ACCOUNT = 1
        CONNECTION = 2
        ABOUT = 3

    def __init__(self):
        super().__init__()  # Call the inherited classes __init__ method
        uic.loadUi('form/configurationdialog.ui', self)  # Load the .ui file

        self._tabName = self.TabName.TOPICS
        self._topicList = []

    def setTopicList(self, topicList):
        self.topicList = topicList
        self.topicListWidget.addItems(self.topicList)
        for i in range(len(self.topicList)):
            topic = self.topicList[i]
            cbName = QtWidgets.QCheckBox(self.topicListWidget)
            cbName.setText(topic.name)
            cbName.setChecked(topic.included)
            self.topicNameGridLayout.addWidget(cbName, i, 0)

            lblType = QtWidgets.QLabel(self.topicListWidget)
            lblType.setText(topic.type)
            self.topicNameGridLayout.addWidget(lblType, i, 1)

            lblDate = QtWidgets.QLabel(self.topicListWidget)
            lblDate.setText(topic.time_created)
            self.topicNameGridLayout.addWidget(lblDate, i, 2)

    def setTab(self, tabName):
        if tabName == self.TabName.TOPICS:
            self.tabWidget.setCurrentWidget(self.TopicsWidget)
        elif tabName == self.TabName.ACCOUNT:
            self.tabWidget.setCurrentWidget(self.AccountWidget)
        elif tabName == self.TabName.CONNECTION:
            self.tabWidget.setCurrentWidget(self.ConnectionWidget)
        elif tabName == self.TabName.ABOUT:
            self.tabWidget.setCurrentWidget(self.AboutWidget)

    def on_comboBox_currentIndexChanged(self, index):
        if index == 0:
            self.frameTopicName.hide()
            self.frameTopicName.show()
            print("switch to topics by name")
        else:
            self.frameTopicName.show()
            self.frameTopicName.hide()
            print("switch to topics by type")
