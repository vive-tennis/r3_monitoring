import sys
import time
from threading import Thread
from r3_core.r3_monitoring_client import R3MonitoringClient
from r3_configs.config_robot import CONFIGS
from r3_core.system_stat import SystemStatLogger

# add this folder to the python path
sys.path.append(".")


def main(with_gui=False):
    r3_monitoring = R3MonitoringClient(CONFIGS)
    system_stat_logger = SystemStatLogger()

    def run_r3_monitoring_thread():
        while True:
            r3_monitoring.step()  # update topics
            r3_monitoring.send_msg(system_stat_logger.create_msg())
            time.sleep(3)

    if with_gui:
        from r3_core.app_controller import AppController
        from PyQt5.QtWidgets import QApplication
        app = QApplication(sys.argv)
        app.setQuitOnLastWindowClosed(False)
        app_controller = AppController(app, r3_monitoring)
        app_controller.make_system_tray_menu()

        # run on a separate thread
        r3_monitoring_thread = Thread(target=run_r3_monitoring_thread)
        r3_monitoring_thread.start()
        app.exec_()
    else:
        run_r3_monitoring_thread()


if __name__ == '__main__':
    main()

