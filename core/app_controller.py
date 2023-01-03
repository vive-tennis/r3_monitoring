from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QMenu, QSystemTrayIcon, QAction, QWidget
from .config_dialog import ConfigDialog


class AppController(QWidget):
    def __init__(self, app_ptr: "QApplication", r3_monitoring_client_ptr: "R3MonitoringClient"):
        super().__init__()
        self.app = app_ptr
        self._r3_monitoring_client = r3_monitoring_client_ptr

        self._configDialog = ConfigDialog()
        self._systemTray = QSystemTrayIcon()
        self.trayIconMenu = QMenu()

        self._serviceName = ""
        self._topicsFile = ""
        self._r3monitoringConfFile = ""
        self._topicList = []

        self.loadSettings()
        # self._confDialog.setTopicList(self._topicList)

    def loadSettings(self):
        pass  # todo

    def make_system_tray_menu(self):
        icon = QIcon("resource/trayicon.jpeg")
        self._systemTray.setIcon(icon)
        self._systemTray.setToolTip("R3 Monitoring")
        self._systemTray.setVisible(True)
        # self.action_ = QAction("A menu item")

#       // create actions
        self.topicsAction = QAction(self.tr("&Topics"))
        self.accountAction = QAction(self.tr("&Account"))
        self.connectionAction = QAction(self.tr("&Connection"))
        self.pauseAction = QAction(self.tr("&Pause Monitoring"))
        self.aboutAction = QAction(self.tr("&About"))
        self.quitAction = QAction(self.tr("&Quit"))

#       // add actions to the menu
        self.trayIconMenu.addAction(self.topicsAction)
        self.trayIconMenu.addAction(self.accountAction)
        self.trayIconMenu.addAction(self.connectionAction)
        self.trayIconMenu.addAction(self.pauseAction)
        self.trayIconMenu.addSeparator()
        self.trayIconMenu.addAction(self.aboutAction)
        self.trayIconMenu.addAction(self.quitAction)

#       // add the menu to the trayicon
        self._systemTray.setContextMenu(self.trayIconMenu)

#       // connect actions to slots
        self.topicsAction.triggered.connect(self.on_topicsAction_triggered)
        self.accountAction.triggered.connect(self.on_accountAction_triggered)
        self.connectionAction.triggered.connect(self.on_connectiontAction_triggered)
        self.pauseAction.triggered.connect(self.on_pauseAction_triggered)
        self.aboutAction.triggered.connect(self.on_aboutAction_triggered)
        self.quitAction.triggered.connect(self.on_quitAction_triggered)

    def on_topicsAction_triggered(self):
        self._configDialog.setTab(ConfigDialog.TabName.TOPICS)
        self._configDialog.setTopicList(self._r3_monitoring_client.all_topics)
        self._configDialog.show()

    def on_accountAction_triggered(self):
        self._configDialog.setTab(ConfigDialog.TabName.ACCOUNT)
        self._configDialog.show()

    def on_connectiontAction_triggered(self):
        self._configDialog.setTab(ConfigDialog.TabName.CONNECTION)
        self._configDialog.show()

    def on_pauseAction_triggered(self):
        if "Pause" in self.pauseAction.text():
            self.pauseAction.setText("Resume Monitoring")
            self._r3_monitoring_client.pause()

        else:
            self.pauseAction.setText("Pause Monitoring")
            self._r3_monitoring_client.resume()

    def on_aboutAction_triggered(self):
        self._configDialog.setTab(ConfigDialog.TabName.ABOUT)
        self._configDialog.show()

    def on_quitAction_triggered(self):
        self.app.quit()
