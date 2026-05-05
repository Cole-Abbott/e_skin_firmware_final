import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pyqtgraph as pg
import usb.backend.libusb1
import usb.core
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


@dataclass(frozen=True)
class USBConfig:
    vendor_id: int = 0x04D8
    product_id: int = 0x0053  # PCB
    endpoint: int = 0x81
    data_len: int = 1536
    adc_samples: int = 766
    adc_period_us: float = 0.16
    libusb_path: str = "/opt/homebrew/opt/libusb/lib/libusb-1.0.dylib"


DEFAULT_USB_CONFIG = USBConfig()
NUM_STREAMS = 6


def init_usb_device(config: USBConfig = DEFAULT_USB_CONFIG):
    backend = usb.backend.libusb1.get_backend(
        find_library=lambda _x: config.libusb_path
    )
    dev = usb.core.find(
        idVendor=config.vendor_id,
        idProduct=config.product_id,
        backend=backend,
    )
    if dev is None:
        raise ValueError("Device not found")

    dev.set_configuration()
    return dev


def read_usb_data(
    dev, config: USBConfig = DEFAULT_USB_CONFIG
) -> tuple[np.ndarray, int]:
    try:
        data = dev.read(config.endpoint, config.data_len, timeout=10)
    except usb.core.USBError as err:
        if err.errno == 60:
            raise ValueError("Operation timed out")
        if err.errno == 19:
            raise ValueError("Device not found")
        raise ValueError(f"USB read error: {err}")

    expected_len = config.adc_samples * 2 + 2
    if len(data) < expected_len:
        raise ValueError(f"Short USB packet: {len(data)} bytes")

    packet_id = int(data[0])
    payload = bytes(data[2 : 2 + config.adc_samples * 2])
    adc_data = np.frombuffer(payload, dtype="<u2").astype(np.float64, copy=False)
    return adc_data, packet_id


class SixStreamPlotWindow(QMainWindow):
    def __init__(
        self,
        dev,
        config: USBConfig = DEFAULT_USB_CONFIG,
        num_streams: int = NUM_STREAMS,
        enable_calibration: bool = False,
        window_title: str = "Live USB ADC Plot (6 Streams)",
    ):
        super().__init__()
        self.dev = dev
        self.config = config
        self.num_streams = num_streams
        self.enable_calibration = enable_calibration

        self.paused = False
        self.last_packet_id: Optional[int] = None
        self.packet_steps = 0
        self.missed_packets = 0

        self.x = np.arange(
            0.0,
            self.config.adc_samples * self.config.adc_period_us,
            self.config.adc_period_us,
        )
        self.stream_data = [
            np.zeros(self.config.adc_samples, dtype=np.float64)
            for _ in range(self.num_streams)
        ]
        self.calibration_data: list[Optional[np.ndarray]] = [None] * self.num_streams
        self.calibration_pending: set[int] = set()
        self.calibration_active = False

        self.setWindowTitle(window_title)
        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self):
        main_widget = QWidget()
        main_layout = QVBoxLayout()

        self.status_label = QLabel("Waiting for first packet...")
        main_layout.addWidget(self.status_label)

        grid = QGridLayout()
        self.curves = []

        for idx in range(self.num_streams):
            graph = pg.PlotWidget()
            graph.setTitle(f"Stream {idx + 1}")
            graph.setLabel("left", "ADC Value")
            graph.setLabel("bottom", "Time (us)")
            curve = graph.plot(pen=pg.mkPen(width=2))
            grid.addWidget(graph, idx // 2, idx % 2)
            self.curves.append(curve)

        main_layout.addLayout(grid)

        button_row = QHBoxLayout()

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        button_row.addWidget(self.pause_button)

        self.reset_sync_button = QPushButton("Reset Stream Sync")
        self.reset_sync_button.clicked.connect(self.reset_sync)
        button_row.addWidget(self.reset_sync_button)

        if self.enable_calibration:
            self.calibrate_button = QPushButton("Calibrate")
            self.calibrate_button.clicked.connect(self.start_calibration)
            button_row.addWidget(self.calibrate_button)

            self.clear_cal_button = QPushButton("Clear Calibration")
            self.clear_cal_button.clicked.connect(self.clear_calibration)
            button_row.addWidget(self.clear_cal_button)

        main_layout.addLayout(button_row)
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

    def _setup_timer(self):
        self.timer = QTimer()
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.setInterval(0)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Resume" if self.paused else "Pause")

    def reset_sync(self):
        self.last_packet_id = None
        self.packet_steps = 0
        self.missed_packets = 0
        self.status_label.setText("Waiting for first packet...")

    def start_calibration(self):
        if self.paused:
            self.toggle_pause()
        self.calibration_data = [None] * self.num_streams
        self.calibration_pending = set(range(self.num_streams))
        self.calibration_active = False
        self.status_label.setText(
            "Calibration started: waiting for one sample from each stream..."
        )

    def clear_calibration(self):
        self.calibration_data = [None] * self.num_streams
        self.calibration_pending.clear()
        self.calibration_active = False
        self.status_label.setText("Calibration cleared.")

    def _maybe_capture_calibration(self, stream_idx: int):
        if stream_idx in self.calibration_pending:
            self.calibration_data[stream_idx] = self.stream_data[stream_idx].copy()
            self.calibration_pending.remove(stream_idx)
            if not self.calibration_pending:
                self.calibration_active = True

    def _calibration_status_text(self) -> str:
        if not self.enable_calibration:
            return ""
        if self.calibration_pending:
            return f" | Cal: collecting ({len(self.calibration_pending)} left)"
        if self.calibration_active:
            return " | Cal: active"
        return " | Cal: inactive"

    def _get_plot_trace(self, stream_idx: int) -> np.ndarray:
        if self.calibration_active and self.calibration_data[stream_idx] is not None:
            return self.stream_data[stream_idx] - self.calibration_data[stream_idx]
        return self.stream_data[stream_idx]

    def update_plot_data(self):
        if self.paused:
            return

        try:
            adc_data, packet_id = read_usb_data(self.dev, self.config)
            stream_idx = packet_id
            self.stream_data[stream_idx] = adc_data

            self._maybe_capture_calibration(stream_idx)

            plot_trace = self._get_plot_trace(stream_idx)
            self.curves[stream_idx].setData(self.x, plot_trace)

            self.status_label.setText(
                f"Packet {packet_id} -> Stream {stream_idx + 1} | "
                f"{self._calibration_status_text()}"
            )
            self.timer.setInterval(0)

        except ValueError as err:
            if str(err) == "Operation timed out":
                print("Operation timed out, retrying...")
                self.timer.setInterval(1)
            elif str(err) == "Device not found":
                print("Device not found, retrying...")
                try:
                    self.dev = init_usb_device(self.config)
                    self.reset_sync()
                    self.timer.setInterval(0)
                    print("Device reinitialized successfully.")
                except ValueError as reinit_err:
                    print(f"Failed to reinitialize device: {reinit_err}")
                    self.timer.setInterval(1000)
            else:
                print(f"Error reading data: {err}")
                self.timer.setInterval(1000)


def run_usb_stream_plot(
    enable_calibration: bool = False,
    window_title: str = "Live USB ADC Plot (6 Streams)",
    config: USBConfig = DEFAULT_USB_CONFIG,
):
    dev = init_usb_device(config)
    app = QApplication(sys.argv)
    window = SixStreamPlotWindow(
        dev=dev,
        config=config,
        enable_calibration=enable_calibration,
        window_title=window_title,
    )
    window.show()
    sys.exit(app.exec_())
