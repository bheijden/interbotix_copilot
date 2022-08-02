from interbotix_copilot.copilot import Copilot
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSlot
from PyQt5 import QtCore
import logging


class CustomQTextEdit(QTextEdit):
    def __init__(self, parent):
        super().__init__(parent)

    @pyqtSlot(str)
    def updateStatus(self, message: str):
        self.append_message(message)

    def append_message(self, message):
        # this "instance" method is very useful!
        app_thread = QApplication.instance().thread()
        curr_thread = QThread.currentThread()
        if app_thread != curr_thread:
            print("attempt to call MainWindow.append_message from non-app thread")
            raise Exception("attempt to call MainWindow.append_message from non-app thread")
        # ms_now = datetime.datetime.now().isoformat(sep=' ', timespec='milliseconds')
        self.append(message)
        # self.widget: QTextEdit.moveCursor(QTextCursor.End)
        # self.messages_text_box.insertPlainText(f'{ms_now}: {message}\n')
        # scroll to bottom
        # self.messages_text_box.moveCursor(QtGui.QTextCursor.End)


# class QTextEditLogger(logging.Handler):
#     def __init__(self, parent):
#         super().__init__()
#         self.widget = CustomQTextEdit(parent)
#         self.widget.setReadOnly(True)
#         self.widget.setLineWrapMode(QTextEdit.NoWrap)
#
#     def emit(self, record):
#         msg = self.format(record)
#         self.widget.append(msg)


class TaskLogger(logging.Handler, QtCore.QObject):
    signalMessage = QtCore.pyqtSignal(str)

    def __init__(self):
        logging.Handler.__init__(self)  # explicit calls without super
        QtCore.QObject.__init__(self)

    @QtCore.pyqtSlot()
    def emit(self, record):
        msg = self.format(record)
        self.signalMessage.emit(msg)


class CopilotGui(QMainWindow):
    def __init__(self, arm: Copilot):
        super().__init__()
        self.arm = arm

        self.setWindowTitle(f"Copilot - {arm.NAME} - {arm.GROUP_NAME}")

        # setting geometry
        self.setGeometry(100, 100, 800, 400)

        # Buttons to disable in case of these events
        self.disable_stop = []
        self.disable_ft = []
        self.disable_torque = []

        # creating a feedthrough button
        self.button_ft = QPushButton("Feedthrough", self)
        self.button_ft.setCheckable(True)  # setting checkable to true
        self.button_ft.clicked.connect(self.toggleFeedthrough)  # setting calling method by button_ft
        self.button_ft.setStyleSheet("background-color : lightgrey")  # setting default color of button_ft to light-grey
        self.disable_stop.append(self.button_ft)
        self.disable_torque.append(self.button_ft)

        # Creating stop button
        self.button_stop = QPushButton("Stop", self)
        self.button_stop.setCheckable(True)  # setting checkable to true
        self.button_stop.clicked.connect(self.toggleStop)
        self.disable_torque.append(self.button_stop)

        # Creating reboot button
        self.button_reboot = QPushButton("Smart Reboot", self)
        self.button_reboot.clicked.connect(lambda: self.arm.reboot(enable=True, smart_reboot=True))
        self.disable_stop.append(self.button_reboot)
        self.disable_ft.append(self.button_reboot)

        # Creating torque toggle button
        self.button_torque = QPushButton("Torque Enabled", self)
        self.button_torque.setStyleSheet("background-color : lightblue")  # setting background color to light-blue
        self.button_torque.setCheckable(True)  # setting checkable to true
        self.button_torque.setChecked(True)
        self.button_torque.clicked.connect(self.toggleTorque)
        self.disable_stop.append(self.button_torque)
        self.disable_ft.append(self.button_torque)

        # Creating go to home button
        self.button_home = QPushButton("Home")
        self.button_home.clicked.connect(self.arm.go_to_home)
        self.disable_stop.append(self.button_home)
        self.disable_ft.append(self.button_home)
        self.disable_torque.append(self.button_home)

        # Creating go to sleep button
        self.button_sleep = QPushButton("Sleep")
        self.button_sleep.clicked.connect(self.arm.go_to_sleep)
        self.disable_stop.append(self.button_sleep)
        self.disable_ft.append(self.button_sleep)
        self.disable_torque.append(self.button_sleep)

        # FINISHED handler
        self.f_worker = TaskLogger()
        self.f_worker.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s', datefmt='%H:%M:%S',))
        self.f_worker.setLevel(logging.INFO)
        self.f_thread = QtCore.QThread()
        self.f_worker.moveToThread(self.f_thread)
        self.f_thread.start()
        f_logTextBox = CustomQTextEdit(self)
        f_logTextBox.setReadOnly(True)
        f_logTextBox.setLineWrapMode(QTextEdit.NoWrap)
        self.f_worker.signalMessage.connect(f_logTextBox.updateStatus)
        self.arm.task_loggers["FINISHED"].addHandler(self.f_worker)

        # Scheduled
        self.s_worker = TaskLogger()
        self.s_worker.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s', datefmt='%H:%M:%S',))
        self.s_worker.setLevel(logging.INFO)
        self.s_thread = QtCore.QThread()
        self.s_worker.moveToThread(self.s_thread)
        self.s_thread.start()
        s_logTextBox = CustomQTextEdit(self)
        s_logTextBox.setReadOnly(True)
        s_logTextBox.setLineWrapMode(QTextEdit.NoWrap)
        self.s_worker.signalMessage.connect(s_logTextBox.updateStatus)
        self.arm.task_loggers["SCHEDULED"].addHandler(self.s_worker)

        # Create layout
        layout = QGridLayout()
        layout.addWidget(self.button_ft, 0, 0)
        layout.addWidget(self.button_stop, 0, 1)
        layout.addWidget(self.button_home, 1, 0)
        layout.addWidget(self.button_reboot, 1, 1)
        layout.addWidget(self.button_sleep, 2, 0)
        layout.addWidget(self.button_torque, 2, 1)
        layout.addWidget(f_logTextBox, 3, 1)
        layout.addWidget(s_logTextBox, 3, 0)
        container = QWidget()
        container.setLayout(layout)

        # Set the central widget of the Window.
        self.setCentralWidget(container)
        self.update()
        self.show()

    def shutdown(self):
        self.f_thread.quit()
        self.s_thread.quit()

    # method called by button
    def toggleFeedthrough(self):
        if self.button_ft.isChecked():  # if button is checked
            # Disable all buttons
            [b.setDisabled(True) for b in self.disable_ft]
            self.button_ft.setStyleSheet("background-color : lightblue")  # setting background color to light-blue
            # Enable feedthrough (will wait for all tasks to finish).
            self.arm.enable_feedthrough()
        else:  # if it is unchecked
            self.button_ft.setStyleSheet("background-color : lightgrey")  # set background color back to light-grey
            self.arm.disable_feedthrough()
            # Enable all buttons
            [b.setEnabled(True) for b in self.disable_ft]

    # method called by button
    def toggleStop(self):
        if self.button_stop.isChecked():  # if button is checked
            self.button_ft.setCheckable(False)
            self.button_ft.setStyleSheet("background-color : lightgrey")  # set background color back to light-grey
            [b.setDisabled(True) for b in self.disable_stop]
            self.arm.disable_feedthrough()  # Disable feedthrough when stop is clicked.
            self.arm.stop()  # Initiate emergency stop
            self.button_stop.setStyleSheet("background-color : red")  # setting background color to light-blue
        else:  # if it is unchecked
            self.arm.proceed()
            self.button_stop.setStyleSheet("background-color : lightgrey")  # set background color back to light-grey
            # Enable all buttons
            [b.setEnabled(True) for b in self.disable_stop]
            self.button_ft.setCheckable(True)

    # method called by button
    def toggleTorque(self):
        if self.button_torque.isChecked():  # if button is checked
            # Enable all buttons
            [b.setEnabled(True) for b in self.disable_torque]
            self.button_torque.setStyleSheet("background-color : lightblue")  # setting background color to light-blue
            self.arm.enable()
            self.button_torque.setText("Torque Enabled")
        else:  # if it is unchecked
            self.button_torque.setStyleSheet("background-color : lightgrey")  # set background color back to light-grey
            # Disable all buttons
            [b.setDisabled(True) for b in self.disable_torque]
            self.arm.disable()
            self.button_torque.setText("Torque Disabled")

