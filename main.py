import sys
import time
from threading import Thread
from PyQt5.QtWidgets import QApplication
from core.app_controller import AppController
from core.r3_monitoring_client import R3MonitoringClient
from core.config import CONFIGS
from core.system_stat import SystemStatLogger

# add this folder to the python path
sys.path.append(".")


def main():
    app = QApplication([])
    app.setQuitOnLastWindowClosed(False)

    r3_monitoring = R3MonitoringClient(CONFIGS)
    system_stat_logger = SystemStatLogger()
    app_controller = AppController(app, r3_monitoring)
    app_controller.make_system_tray_menu()

    # run on a separate thread
    def r3_monitoring_thread_run():
        while True:
            r3_monitoring.step()  # update topics
            r3_monitoring.send_msg(system_stat_logger.create_msg())
            time.sleep(3)
    r3_monitoring_thread = Thread(target=r3_monitoring_thread_run)
    r3_monitoring_thread.start()

    app.exec_()


if __name__ == '__main__':
    main()

