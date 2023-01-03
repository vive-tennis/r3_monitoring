import sys
import time
from threading import Thread
from PyQt5.QtWidgets import QApplication
from core.app_controller import AppController
from core.r3_monitoring_client import R3MonitoringClient
from core.config import CONFIGS

# add this folder to the python path
sys.path.append(".")


def main():
    app = QApplication([])
    app.setQuitOnLastWindowClosed(False)

    r3_monitoring = R3MonitoringClient(CONFIGS)
    app_controller = AppController(app, r3_monitoring)
    app_controller.make_system_tray_menu()

    # run on a separate thread
    def r3_monitoring_thread_run():
        while True:
            r3_monitoring.step()  # update topics
            time.sleep(3)
    r3_monitoring_thread = Thread(target=r3_monitoring_thread_run)
    r3_monitoring_thread.start()

    app.exec_()


if __name__ == '__main__':
    main()

