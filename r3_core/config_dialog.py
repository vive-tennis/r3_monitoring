from PyQt5 import QtWidgets, uic
from typing import Dict


class ConfigDialog(QtWidgets.QDialog):
    class TabName:
        TOPICS = 0
        ACCOUNT = 1
        CONNECTION = 2
        ABOUT = 3

    def __init__(self):
        super().__init__()  # Call the inherited classes __init__ method
        uic.loadUi('form/configurationdialog.ui', self)  # Load the .ui file

        self.cbSelectTopics.setCurrentIndex(0);
        self.stwTopics.setCurrentIndex(0);

        self.cbSelectTopics.currentIndexChanged.connect(self.onCbSelectTopicsCurrentIndexChanged)

        self._tabName = self.TabName.TOPICS
        self._name_checkboxes = {}
        self._type_labels = {}
        self._date_labels = {}

    def setTopicList(self, topicList: Dict):
        # self.topicListWidget.addItems(topicList)
        for i, topic in enumerate(topicList.items()):
            cbName = QtWidgets.QCheckBox(self.TopicsWidget)
            cbName.setText(topic[0])
            cbName.setChecked(True)
            self.topicNameGridLayout.addWidget(cbName, i, 0)
            self._name_checkboxes[topic[0]] = cbName

            lblType = QtWidgets.QLabel(self.TopicsWidget)
            lblType.setText(topic[1])
            self.topicNameGridLayout.addWidget(lblType, i, 1)
            self._type_labels[topic[0]] = lblType

            lblDate = QtWidgets.QLabel(self.TopicsWidget)
            lblDate.setText("topic.time_created")
            self.topicNameGridLayout.addWidget(lblDate, i, 2)
            self._date_labels[topic[0]] = lblDate

    def setTab(self, tabName):
        if tabName == self.TabName.TOPICS:
            self.tabWidget.setCurrentWidget(self.TopicsWidget)
        elif tabName == self.TabName.ACCOUNT:
            self.tabWidget.setCurrentWidget(self.AccountWidget)
        elif tabName == self.TabName.CONNECTION:
            self.tabWidget.setCurrentWidget(self.ConnectionWidget)
        elif tabName == self.TabName.ABOUT:
            self.tabWidget.setCurrentWidget(self.AboutWidget)

    def onCbSelectTopicsCurrentIndexChanged(self, index):
        if index == 0:
            print("switch to topics by name")
        else:
            print("switch to topics by type")
        self.stwTopics.setCurrentIndex(index)
