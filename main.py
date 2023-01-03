from PyQt5.QtWidgets import QApplication
from python.app_controller import AppController


def main():
    app = QApplication([])
    app.setQuitOnLastWindowClosed(False)

    app_controller = AppController(app)
    app_controller.make_system_tray_menu()

    app.exec_()


if __name__ == '__main__':
    main()

