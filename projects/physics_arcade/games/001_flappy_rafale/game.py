"""Flappy Rafale: a simple IMU pitch-controlled PySide6 game."""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import random
import sys
import time

import numpy as np
import serial
from serial.tools import list_ports
from PySide6 import QtCore, QtGui, QtWidgets


PROJECT_ROOT = Path(__file__).resolve().parents[4]
SOURCE_ROOT = PROJECT_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from imu_lab_pro.orientation import (  # noqa: E402
    AXIS_VECTORS,
    AttitudeEstimator,
    sensor_mount_rotation,
)
from physics import (  # noqa: E402
    PitchControlSettings,
    circle_hits_gate,
    clamp,
    pitch_command,
    update_vertical_speed,
)


class ImuInput(QtCore.QObject):
    attitude_changed = QtCore.Signal(float, bool, bool, str)
    connection_changed = QtCore.Signal(bool, str)

    def __init__(self, mounting, baud: int = 460800, parent=None):
        super().__init__(parent)
        self.baud = baud
        self.serial_port = None
        self.partial = b""
        self.estimator = AttitudeEstimator(sensor_to_aircraft=mounting)
        self.accel_range = 2
        self.rate_hz = 200
        self.dlpf = 3
        self.gyro_range = 250
        self.last_data_time = None

    @property
    def connected(self) -> bool:
        return self.serial_port is not None

    def connect_port(self, port: str) -> None:
        self.disconnect_port()
        try:
            device = serial.Serial(port, self.baud, timeout=0)
            device.reset_input_buffer()
            device.write(b"CFG 2 200 3 250\n")
        except (serial.SerialException, OSError) as exc:
            self.connection_changed.emit(False, f"Connection failed: {exc}")
            return
        self.serial_port = device
        self.partial = b""
        self.estimator.reset()
        self.last_data_time = None
        self.connection_changed.emit(
            True, f"Connected to {device.port}; hold the sensor still")

    def disconnect_port(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (serial.SerialException, OSError):
                pass
        self.serial_port = None
        self.partial = b""

    def poll(self) -> None:
        if self.serial_port is None:
            return
        try:
            available = min(self.serial_port.in_waiting, 250_000)
            if not available:
                return
            self.partial += self.serial_port.read(available)
        except (serial.SerialException, OSError) as exc:
            self.disconnect_port()
            self.connection_changed.emit(False, f"Serial error: {exc}")
            return

        lines = self.partial.split(b"\n")
        self.partial = lines.pop()
        if len(self.partial) > 4096:
            self.partial = b""
        for raw in lines:
            self._handle_line(raw.decode("ascii", errors="replace").strip())

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        if line.startswith("# CONFIG "):
            fields = line.split()
            if len(fields) == 7 and fields[-1] == "OK":
                try:
                    values = tuple(map(int, fields[2:6]))
                except ValueError:
                    return
                (self.accel_range, self.rate_hz, self.dlpf,
                 self.gyro_range) = values
                self.estimator.reset()
            return
        if line.startswith("#"):
            return

        fields = line.split(",")
        if len(fields) != 8:
            return
        try:
            _, t_us, ax, ay, az, gx, gy, gz = map(int, fields)
        except ValueError:
            return

        accel_scale = {
            2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0
        }[self.accel_range]
        gyro_scale = {
            250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4
        }[self.gyro_range]
        acceleration = np.array([ax, ay, az], dtype=float) / accel_scale
        gyro = np.array([gx, gy, gz], dtype=float) / gyro_scale
        state = self.estimator.update(t_us, acceleration, gyro)
        self.last_data_time = time.monotonic()
        self.attitude_changed.emit(
            state.pitch_deg, state.bias_ready, state.stationary, state.status)


@dataclass
class Gate:
    x: float
    gap_center: float
    gap_size: float
    width: float = 86.0
    scored: bool = False

    @property
    def gap_top(self) -> float:
        return self.gap_center - self.gap_size / 2.0

    @property
    def gap_bottom(self) -> float:
        return self.gap_center + self.gap_size / 2.0


class GameCanvas(QtWidgets.QWidget):
    game_ended = QtCore.Signal(int)
    score_changed = QtCore.Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(760, 480)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.settings = PitchControlSettings()
        self.current_pitch = 0.0
        self.neutral_pitch = None
        self.relative_pitch = 0.0
        self.command = 0.0
        self.plane_x = 170.0
        self.plane_y = 240.0
        self.vertical_speed = 0.0
        self.scroll_speed = 225.0
        self.gate_spacing = 390.0
        self.gates = []
        self.score = 0
        self.running = False
        self.game_over = False
        self.random = random.Random()
        self.elapsed = QtCore.QElapsedTimer()
        self.elapsed.start()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._frame)
        self.timer.start(16)

    def set_pitch(self, pitch_deg: float) -> None:
        self.current_pitch = float(pitch_deg)

    def capture_neutral(self) -> None:
        self.neutral_pitch = self.current_pitch
        self.relative_pitch = 0.0
        self.command = 0.0
        self.update()

    def clear_neutral(self) -> None:
        self.neutral_pitch = None
        self.relative_pitch = 0.0
        self.command = 0.0
        self.vertical_speed = 0.0
        self.update()

    def reset_game(self) -> None:
        height = max(self.height(), 480)
        self.plane_y = height / 2.0
        self.vertical_speed = 0.0
        self.score = 0
        self.game_over = False
        self.gates = []
        start_x = max(self.width() * 0.72, self.plane_x + 300.0)
        for index in range(3):
            self.gates.append(self._new_gate(start_x + index * self.gate_spacing))
        self.score_changed.emit(0)
        self.update()

    def set_running(self, running: bool) -> None:
        if running and self.game_over:
            self.reset_game()
        self.running = bool(running)
        self.elapsed.restart()
        self.update()

    def _new_gate(self, x: float) -> Gate:
        height = max(self.height(), 480)
        gap_size = clamp(height * 0.36, 165.0, 215.0)
        margin = gap_size / 2.0 + 55.0
        center = self.random.uniform(margin, height - margin)
        return Gate(x=x, gap_center=center, gap_size=gap_size)

    def _frame(self) -> None:
        dt = min(self.elapsed.restart() / 1000.0, 0.05)
        if not self.running:
            self.update()
            return

        if self.neutral_pitch is not None:
            self.command, self.relative_pitch = pitch_command(
                self.current_pitch, self.neutral_pitch, self.settings)
        self.vertical_speed = update_vertical_speed(
            self.vertical_speed, self.command, dt, self.settings)
        self.plane_y += self.vertical_speed * dt

        for gate in self.gates:
            gate.x -= self.scroll_speed * dt
            if not gate.scored and gate.x + gate.width < self.plane_x:
                gate.scored = True
                self.score += 1
                self.score_changed.emit(self.score)

        while self.gates and self.gates[0].x + self.gates[0].width < -20.0:
            self.gates.pop(0)
            last_x = self.gates[-1].x if self.gates else self.width()
            self.gates.append(self._new_gate(last_x + self.gate_spacing))

        radius = 23.0
        collision = self.plane_y - radius < 0 or self.plane_y + radius > self.height()
        if not collision:
            collision = any(
                circle_hits_gate(
                    self.plane_x, self.plane_y, radius,
                    gate.x, gate.width, gate.gap_top, gate.gap_bottom)
                for gate in self.gates
            )
        if collision:
            self.running = False
            self.game_over = True
            self.game_ended.emit(self.score)
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self.gates:
            self.reset_game()

    def paintEvent(self, event):
        del event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        gradient = QtGui.QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0.0, QtGui.QColor("#071426"))
        gradient.setColorAt(1.0, QtGui.QColor("#123456"))
        painter.fillRect(self.rect(), gradient)

        self._draw_grid(painter)
        for gate in self.gates:
            self._draw_gate(painter, gate)
        self._draw_aircraft(painter)
        self._draw_hud(painter)

        if not self.running:
            self._draw_center_message(painter)
        painter.end()

    def _draw_grid(self, painter: QtGui.QPainter) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(64, 105, 140, 45), 1))
        for x in range(0, self.width(), 80):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), 80):
            painter.drawLine(0, y, self.width(), y)

    def _draw_gate(self, painter: QtGui.QPainter, gate: Gate) -> None:
        body = QtGui.QColor("#1fa37a")
        edge = QtGui.QColor("#6ee7b7")
        painter.setPen(QtGui.QPen(edge, 2))
        painter.setBrush(body)
        top = QtCore.QRectF(gate.x, 0, gate.width, max(0.0, gate.gap_top))
        bottom = QtCore.QRectF(
            gate.x, gate.gap_bottom, gate.width,
            max(0.0, self.height() - gate.gap_bottom))
        painter.drawRoundedRect(top, 7, 7)
        painter.drawRoundedRect(bottom, 7, 7)

    def _draw_aircraft(self, painter: QtGui.QPainter) -> None:
        painter.save()
        painter.translate(self.plane_x, self.plane_y)
        painter.rotate(-clamp(self.relative_pitch, -24.0, 24.0))

        path = QtGui.QPainterPath()
        path.moveTo(34.0, 0.0)
        path.lineTo(10.0, -7.0)
        path.lineTo(-8.0, -24.0)
        path.lineTo(-15.0, -22.0)
        path.lineTo(-9.0, -6.0)
        path.lineTo(-31.0, -4.0)
        path.lineTo(-36.0, 0.0)
        path.lineTo(-31.0, 4.0)
        path.lineTo(-9.0, 6.0)
        path.lineTo(-15.0, 22.0)
        path.lineTo(-8.0, 24.0)
        path.lineTo(10.0, 7.0)
        path.closeSubpath()

        painter.setPen(QtGui.QPen(QtGui.QColor("#e5e7eb"), 2))
        painter.setBrush(QtGui.QColor("#6b7280"))
        painter.drawPath(path)
        painter.setBrush(QtGui.QColor("#67e8f9"))
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawEllipse(QtCore.QPointF(10.0, -1.5), 7.0, 3.5)
        painter.restore()

    def _draw_hud(self, painter: QtGui.QPainter) -> None:
        painter.setPen(QtGui.QColor("#e2e8f0"))
        font = painter.font()
        font.setPointSize(12)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(18, 30, f"SCORE  {self.score}")
        font.setPointSize(10)
        font.setBold(False)
        painter.setFont(font)
        neutral = "not set" if self.neutral_pitch is None else f"{self.neutral_pitch:+.1f}°"
        painter.drawText(
            18, 53,
            f"Pitch {self.current_pitch:+.1f}°   Neutral {neutral}   "
            f"Relative {self.relative_pitch:+.1f}°")

    def _draw_center_message(self, painter: QtGui.QPainter) -> None:
        if self.game_over:
            title = "FLIGHT ENDED"
            subtitle = "Select Restart, then Start"
        elif self.neutral_pitch is None:
            title = "CALIBRATE THE CONTROLLER"
            subtitle = "Hold the sensor comfortably and select Set neutral"
        else:
            title = "READY"
            subtitle = "Tilt nose-up to climb and nose-down to descend"
        painter.setPen(QtGui.QColor("#f8fafc"))
        font = painter.font()
        font.setPointSize(20)
        font.setBold(True)
        painter.setFont(font)
        title_rect = QtCore.QRectF(0, self.height() * 0.40, self.width(), 40)
        painter.drawText(title_rect, QtCore.Qt.AlignmentFlag.AlignCenter, title)
        font.setPointSize(11)
        font.setBold(False)
        painter.setFont(font)
        subtitle_rect = QtCore.QRectF(0, self.height() * 0.48, self.width(), 30)
        painter.drawText(subtitle_rect, QtCore.Qt.AlignmentFlag.AlignCenter, subtitle)


class GameWindow(QtWidgets.QWidget):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("Physics Arcade | Flappy Rafale")
        self.resize(1120, 700)

        mounting = sensor_mount_rotation(args.sensor_forward, args.sensor_up)
        self.imu = ImuInput(mounting, baud=args.baud, parent=self)
        self.bias_ready = False
        self.sensor_stationary = False
        self.current_pitch = 0.0

        root = QtWidgets.QVBoxLayout(self)
        controls = QtWidgets.QHBoxLayout()
        self.port = QtWidgets.QComboBox()
        self.port.setEditable(True)
        self.port.addItems([item.device for item in list_ports.comports()])
        self.port.setCurrentText(args.port)
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.neutral_button = QtWidgets.QPushButton("Set neutral")
        self.start_button = QtWidgets.QPushButton("Start")
        self.restart_button = QtWidgets.QPushButton("Restart")
        self.score_label = QtWidgets.QLabel("Score: 0")
        self.neutral_button.setToolTip(
            "Available after the first valid IMU sample")
        self.start_button.setToolTip(
            "Available after a neutral controller orientation is captured")
        self.neutral_button.setEnabled(False)
        self.start_button.setEnabled(False)

        for label, widget in (
            ("Port", self.port),
            ("", self.connect_button),
            ("", self.neutral_button),
            ("", self.start_button),
            ("", self.restart_button),
        ):
            if label:
                controls.addWidget(QtWidgets.QLabel(label))
            controls.addWidget(widget)
        controls.addStretch()
        controls.addWidget(self.score_label)
        root.addLayout(controls)

        self.canvas = GameCanvas()
        root.addWidget(self.canvas, 1)
        self.status = QtWidgets.QLabel(
            "Connect the IMU, hold it still, then set a neutral pitch")
        root.addWidget(self.status)

        self.connect_button.clicked.connect(self._toggle_connection)
        self.neutral_button.clicked.connect(self._set_neutral)
        self.start_button.clicked.connect(self._toggle_start)
        self.restart_button.clicked.connect(self._restart)
        self.imu.attitude_changed.connect(self._attitude_changed)
        self.imu.connection_changed.connect(self._connection_changed)
        self.canvas.score_changed.connect(
            lambda score: self.score_label.setText(f"Score: {score}"))
        self.canvas.game_ended.connect(self._game_ended)

        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.imu.poll)
        self.poll_timer.start(5)
        self.watchdog = QtCore.QTimer(self)
        self.watchdog.timeout.connect(self._check_data_timeout)
        self.watchdog.start(100)

    def _toggle_connection(self) -> None:
        if self.imu.connected:
            self.canvas.set_running(False)
            self.imu.disconnect_port()
            self._connection_changed(False, "Disconnected")
            return
        port = self.port.currentText().strip()
        if not port:
            self.status.setText("Enter a serial port")
            return
        self.imu.connect_port(port)

    def _connection_changed(self, connected: bool, message: str) -> None:
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.status.setText(message)
        self.canvas.set_running(False)
        self.canvas.clear_neutral()
        self.start_button.setText("Start")
        self.neutral_button.setEnabled(False)
        self.start_button.setEnabled(False)
        if not connected:
            self.bias_ready = False
            self.sensor_stationary = False

    def _attitude_changed(self, pitch: float, bias_ready: bool,
                          stationary: bool,
                          estimator_status: str) -> None:
        self.current_pitch = pitch
        self.bias_ready = bias_ready
        self.sensor_stationary = stationary
        self.canvas.set_pitch(pitch)
        self.neutral_button.setEnabled(
            self.imu.connected and self.imu.last_data_time is not None)
        self.start_button.setEnabled(
            self.imu.connected and self.canvas.neutral_pitch is not None)
        if not self.canvas.running:
            if self.canvas.neutral_pitch is not None:
                self.status.setText("Neutral set; select Start when ready")
            elif not bias_ready:
                self.status.setText(
                    "IMU data received; set neutral when comfortable "
                    "(gyro calibration continues)")
            elif bias_ready and not stationary:
                self.status.setText(
                    "Gyro calibrated; hold steady for the best neutral reference")
            elif bias_ready:
                self.status.setText("Ready: select Set neutral")
            else:
                self.status.setText(estimator_status)

    def _set_neutral(self) -> None:
        if not self.imu.connected:
            self.status.setText("Connect the IMU first")
            return
        if (self.imu.last_data_time is None
                or time.monotonic() - self.imu.last_data_time > 0.75):
            self.status.setText("Waiting for a current IMU sample")
            return
        self.canvas.capture_neutral()
        self.start_button.setEnabled(True)
        calibration_note = "" if self.bias_ready else " (gyro calibration continues)"
        self.status.setText(
            f"Neutral pitch captured at {self.current_pitch:+.1f}°; "
            f"select Start{calibration_note}")

    def _toggle_start(self) -> None:
        if not self.imu.connected:
            self.status.setText("Connect the IMU first")
            return
        if self.canvas.neutral_pitch is None:
            self.status.setText("Set the neutral pitch before starting")
            return
        if (self.imu.last_data_time is None
                or time.monotonic() - self.imu.last_data_time > 0.75):
            self.status.setText("Waiting for current IMU data")
            return
        running = not self.canvas.running
        self.canvas.set_running(running)
        self.start_button.setText("Pause" if running else "Start")
        self.status.setText("Flying" if running else "Paused")

    def _restart(self) -> None:
        self.canvas.set_running(False)
        self.canvas.reset_game()
        self.start_button.setText("Start")
        self.status.setText("Ready with the current neutral pitch")

    def _game_ended(self, score: int) -> None:
        self.start_button.setText("Start")
        self.status.setText(f"Flight ended with score {score}")

    def _check_data_timeout(self) -> None:
        if not self.canvas.running or self.imu.last_data_time is None:
            return
        if time.monotonic() - self.imu.last_data_time > 0.75:
            self.canvas.set_running(False)
            self.start_button.setText("Start")
            self.status.setText("Paused: IMU data timeout")

    def closeEvent(self, event):
        self.imu.disconnect_port()
        event.accept()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument(
        "--sensor-forward", choices=AXIS_VECTORS, default="+X",
        help="signed sensor axis pointing toward the controller nose")
    parser.add_argument(
        "--sensor-up", choices=AXIS_VECTORS, default="+Z",
        help="signed sensor axis pointing upward")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        sensor_mount_rotation(args.sensor_forward, args.sensor_up)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(
        "QWidget{background:#08111f;color:#e2e8f0;font-size:12px}"
        "QPushButton,QComboBox{background:#17263b;border:1px solid #36506e;"
        "border-radius:5px;padding:7px}"
        "QPushButton:hover{background:#23405f}"
        "QPushButton:disabled{background:#0d1725;color:#52657a;"
        "border-color:#1d2b3d}"
    )
    window = GameWindow(args)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
