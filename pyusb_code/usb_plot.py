import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

import usb.core
import usb.util
import usb.backend.libusb1
import time

last_ID = 0
ADC_SAMPLES = 766
ADC_PERIOD = 0.16 # [us]


def init_usb_device():
    # Adjust path to libusb as needed
    path_to_libusb = '/opt/homebrew/opt/libusb/lib/libusb-1.0.dylib'
    backend = usb.backend.libusb1.get_backend(find_library=lambda x: path_to_libusb)

    dev = usb.core.find(idVendor=0x04d8, idProduct=0x0053, backend=backend)

    if dev is None:
        raise ValueError('Device not found')

    dev.set_configuration()
    return dev

def read_usb_data(dev):
    global last_ID
    # dev.write(0x01, b'\x81', 1000)
    try:
        data = dev.read(0x81, 1536, timeout=10)
    except usb.core.USBError as e:
        if e.errno == 60:
            #operation timed out
            raise ValueError("Operation timed out")
        elif e.errno == 19:
            raise ValueError("Device not found")

    channel_ID = data[0]
    packet_ID = data[1]

    last_ID = packet_ID

    adc_data = np.zeros(ADC_SAMPLES)
    for i in range(0, ADC_SAMPLES):
        adc_data[i] =  data[i * 2 + 2] + (data[i * 2 + 3] << 8)
    return adc_data

class LivePlot(QMainWindow):
    def __init__(self, dev):
        super().__init__()
        self.dev = dev
        self.paused = False

        self.setWindowTitle("Live USB ADC Plot")

        # Main layout widget
        main_widget = QWidget()
        layout = QVBoxLayout()

        # Plot widget
        self.graphWidget = pg.PlotWidget()
        self.plot = self.graphWidget.plot(pen=pg.mkPen(color='c', width=2))
        self.graphWidget.setTitle("ADC Data")
        self.graphWidget.setLabel('left', 'ADC Value')
        self.graphWidget.setLabel('bottom', 'Time (us)')
        layout.addWidget(self.graphWidget)

        # Pause button
        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        layout.addWidget(self.pause_button)

        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

        # Data arrays
        self.x = np.arange(0,ADC_SAMPLES*ADC_PERIOD, ADC_PERIOD)
        self.y = np.zeros(ADC_SAMPLES)

        # Timer
        self.timer = QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Resume" if self.paused else "Pause")

    def update_plot_data(self):
        if self.paused:
            return

        try:
            self.y = read_usb_data(self.dev)
            self.plot.setData(self.x, self.y)
            self.timer.setInterval(1)  # Fast polling when working
        except ValueError as e:
            if str(e) == "Operation timed out":
                print("Operation timed out, retrying...")
                self.timer.setInterval(1000)
            elif str(e) == "Device not found":
                print("Device not found, retrying...")
                # Reinitialize the USB device
                try:
                    self.dev = init_usb_device()
                    self.timer.setInterval(1)  # Reset to fast polling
                    print("Device reinitialized successfully.")
                except ValueError as e:
                    print(f"Failed to reinitialize device: {e}")
                    self.timer.setInterval(1000)
                return
            else:
                print(f"Error reading data: {e}")
                self.timer.setInterval(1000)


if __name__ == '__main__':
    dev = init_usb_device()
    app = QApplication(sys.argv)
    window = LivePlot(dev)
    window.show()
    sys.exit(app.exec_())
