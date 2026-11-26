import argparse
import csv
import time
import sys
from pathlib import Path
from collections import deque
import numpy as np
import serial
from serial.tools import list_ports
from scipy.signal import welch
from scipy.spatial.transform import Rotation as R
from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter
import pyvista as pv
from pyvistaqt import QtInteractor
import trimesh
from vtkmodules.vtkCommonMath import vtkMatrix4x4

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = PROJECT_ROOT / 'src'
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from imu_lab_pro.orientation import (  # noqa: E402
    AXIS_VECTORS,
    AttitudeEstimator,
    aerospace_from_rotation,
    rotation_from_aerospace,
    sensor_mount_rotation,
)

parser = argparse.ArgumentParser()
parser.add_argument('--port', default='COM4')
parser.add_argument('--baud', type=int, default=460800)
parser.add_argument('--sensor-forward', choices=AXIS_VECTORS, default='+X',
                    help='signed sensor axis pointing toward the aircraft nose')
parser.add_argument('--sensor-up', choices=AXIS_VECTORS, default='+Z',
                    help='signed sensor axis pointing toward the aircraft top')
args = parser.parse_args()
pg.setConfigOptions(background='#111c2e',
                    foreground='#cbd5e1', antialias=False)


class Aircraft(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.raw = R.identity()
        self.reference = R.identity()
        self.heading_reference = 0.0
        self.model_rotation = R.identity()  # model nose +X, Z up
        self.sensor_to_aircraft = R.identity()
        self.stationary = False
        self.orientation_status = 'Waiting for IMU data'
        self.actors = []
        self.axis_groups = {}
        self.model_size = 10.0
        lay = QtWidgets.QVBoxLayout(self)
        self.view = QtInteractor(self)
        self.view.set_background('#111c2e')
        lay.addWidget(self.view, 1)
        self.readout = QtWidgets.QLabel(
            'Roll 0.0 | Pitch 0.0 | Yaw 0.0 (relative)')
        lay.addWidget(self.readout)
        self.orientation_note = QtWidgets.QLabel(self.orientation_status)
        lay.addWidget(self.orientation_note)
        controls = QtWidgets.QHBoxLayout()
        self.checks = {}
        for name in ('World XYZ', 'Aircraft XYZ', 'Sensor XYZ'):
            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(True)
            cb.toggled.connect(self.visibility)
            controls.addWidget(cb)
            self.checks[name] = cb
        for name, callback in [('Level / zero', self.level), ('Reset heading', self.reset_heading)]:
            b = QtWidgets.QPushButton(name)
            b.clicked.connect(callback)
            controls.addWidget(b)
        self.camera = QtWidgets.QComboBox()
        self.camera.addItems(
            ['Isometric', 'Top', 'Front', 'Rear', 'Right', 'Left'])
        self.camera.currentTextChanged.connect(self.set_camera)
        controls.addWidget(self.camera)
        lay.addLayout(controls)
        self.load_model()
        self.build_axes()
        self.set_camera('Isometric')
        self.set_attitude(0, 0, 0)

    @staticmethod
    def matrix(rotation, translation=None):
        m = vtkMatrix4x4()
        mat = rotation.as_matrix()
        for i in range(3):
            for j in range(3):
                m.SetElement(i, j, float(mat[i, j]))
        if translation is not None:
            for i in range(3):
                m.SetElement(i, 3, float(translation[i]))
        return m

    def load_model(self):
        path = Path(__file__).resolve().parent / 'models' / 'rafale_style.glb'
        if not path.exists():
            raise FileNotFoundError(f'Missing aircraft GLB: {path}')
        scene = trimesh.load(str(path), force='scene')
        all_parts = [g for g in scene.dump() if isinstance(
            g, trimesh.Trimesh) and len(g.faces)]
        if not all_parts:
            raise RuntimeError('GLB contains no triangle meshes')
        verts = np.vstack([g.vertices for g in all_parts])
        center = (verts.min(axis=0) + verts.max(axis=0)) / 2
        self.model_size = float(np.max(verts.max(axis=0) - verts.min(axis=0)))
        for g in all_parts:
            faces = np.column_stack(
                (np.full(len(g.faces), 3), g.faces)).ravel()
            mesh = pv.PolyData(np.asarray(g.vertices) - center, faces)
            actor = self.view.add_mesh(mesh, color='#b8c0ca', smooth_shading=True,
                                       metallic=0.18, roughness=0.65)
            self.actors.append(actor)
        self.view.add_light(pv.Light(position=(15, -10, 25), intensity=0.8))

    def build_axes(self):
        L = self.model_size * 0.32
        axes = [(1, 0, 0, '#ef4444'), (0, 1, 0, '#22c55e'),
                (0, 0, 1, '#38bdf8')]
        for group in ('World XYZ', 'Aircraft XYZ', 'Sensor XYZ'):
            actors = []
            origin = (-L * 2, -L * 2, -
                      L) if group == 'World XYZ' else (0, 0, 0)
            length = L * (0.8 if group == 'World XYZ' else 0.65)
            for x, y, z, color in axes:
                direction = np.array([x, y, z], dtype=float)
                arrow = pv.Arrow(start=origin, direction=direction, scale=length,
                                 tip_length=0.18, shaft_radius=0.018, tip_radius=0.06)
                actors.append(self.view.add_mesh(arrow, color=color))
            self.axis_groups[group] = actors
        self.visibility()

    def visibility(self):
        for name, actors in self.axis_groups.items():
            visible = self.checks[name].isChecked()
            for actor in actors:
                actor.SetVisibility(visible)
        self.view.render()

    def set_attitude(self, roll, pitch, yaw):
        """Compatibility helper using the documented aerospace convention."""
        self.set_orientation(
            rotation_from_aerospace(roll, pitch, yaw),
            status=self.orientation_status,
        )

    def set_orientation(self, orientation, sensor_to_aircraft=None,
                        stationary=False, status=''):
        self.raw = orientation
        self.sensor_to_aircraft = (
            R.identity() if sensor_to_aircraft is None
            else sensor_to_aircraft
        )
        self.stationary = bool(stationary)
        self.orientation_status = status
        self.refresh()

    def level(self):
        if not self.stationary:
            self.orientation_note.setText(
                'Level rejected: hold the sensor stationary near 1 g')
            return
        self.reference = self.raw
        self.heading_reference = 0.0
        self.orientation_status = 'Level reference captured'
        self.refresh()

    def reset_heading(self):
        relative = self.reference.inv() * self.raw
        yaw, _, _ = relative.as_euler('ZYX', degrees=True)
        self.heading_reference = yaw
        self.refresh()

    def refresh(self):
        relative = self.reference.inv() * self.raw
        # Heading correction in reference coordinates, without erasing roll/pitch.
        heading = R.from_euler('Z', -self.heading_reference, degrees=True)
        shown = heading * relative
        body = shown * self.model_rotation
        body_mat = self.matrix(body)
        for actor in self.actors + self.axis_groups.get('Aircraft XYZ', []):
            actor.SetUserMatrix(body_mat)
        sensor_mat = self.matrix(shown * self.sensor_to_aircraft)
        for actor in self.axis_groups.get('Sensor XYZ', []):
            actor.SetUserMatrix(sensor_mat)
        roll, pitch, yaw = aerospace_from_rotation(shown)
        self.readout.setText(
            f'Roll {roll:+.1f}°   Pitch {pitch:+.1f}°   Yaw {yaw:+.1f}° (relative; yaw drifts)')
        self.orientation_note.setText(self.orientation_status)
        self.view.render()

    def set_camera(self, name):
        d = self.model_size * 2.3
        views = {
            'Isometric': ((d, -d, d * .75), (0, 0, 0), (0, 0, 1)),
            'Top': ((0, 0, d), (0, 0, 0), (1, 0, 0)),
            'Front': ((d, 0, 0), (0, 0, 0), (0, 0, 1)),
            'Rear': ((-d, 0, 0), (0, 0, 0), (0, 0, 1)),
            # Aircraft frame is +X forward, +Y left, +Z up.
            'Right': ((0, -d, 0), (0, 0, 0), (0, 0, 1)),
            'Left': ((0, d, 0), (0, 0, 0), (0, 0, 1)),
        }
        self.view.camera_position = views[name]
        self.view.reset_camera_clipping_range()
        self.view.render()


class Lab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('IMU LAB PRO | Rafale 3D')
        self.resize(1500, 900)
        self.ser = None
        self.file = None
        self.writer = None
        self.pending = None
        self.partial = b''
        self.paused = False
        self.range, self.rate, self.dlpf, self.gyro_range = 2, 200, 3, 250
        try:
            mounting = sensor_mount_rotation(args.sensor_forward, args.sensor_up)
        except ValueError as exc:
            parser.error(str(exc))
        self.estimator = AttitudeEstimator(sensor_to_aircraft=mounting)
        self.roll = self.pitch = self.yaw = 0.0
        self.orientation = R.identity()
        self.orientation_stationary = False
        self.orientation_status = 'Waiting for IMU data'
        self.reset_data()
        root = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QHBoxLayout()
        title.addWidget(QtWidgets.QLabel('IMU LAB / PRO  |  RAFALE 3D'))
        title.addStretch()
        self.status = QtWidgets.QLabel('DISCONNECTED')
        title.addWidget(self.status)
        root.addLayout(title)
        row = QtWidgets.QHBoxLayout()
        self.port = self.combo(
            [x.device for x in list_ports.comports()], args.port, editable=True)
        self.connection = QtWidgets.QPushButton('Connect')
        self.connection.clicked.connect(self.toggle_connection)
        self.g = self.combo(['2', '4', '8', '16'], '2')
        self.hz = self.combo(['50', '100', '200', '500', '1000'], '200')
        self.filt = self.combo(['0 · 260 Hz', '1 · 184 Hz', '2 · 94 Hz',
                               '3 · 44 Hz', '4 · 21 Hz', '5 · 10 Hz', '6 · 5 Hz'], '3 · 44 Hz')
        self.gyro = self.combo(['250', '500', '1000', '2000'], '250')
        apply = QtWidgets.QPushButton('Apply settings')
        apply.clicked.connect(self.apply_config)
        for label, widget in [('Port', self.port), ('', self.connection), ('Accel ±g', self.g), ('Rate Hz', self.hz), ('DLPF', self.filt), ('Gyro ±°/s', self.gyro), ('', apply)]:
            if label:
                row.addWidget(QtWidgets.QLabel(label))
            row.addWidget(widget)
        root.addLayout(row)
        metrics_row = QtWidgets.QHBoxLayout()
        self.metrics = {}
        for name in ['Read Hz', 'Jitter SD', 'Sequence gaps', 'Accel RMS X', 'Peak |a|', 'Accel clips', 'Gyro clips']:
            box = QtWidgets.QGroupBox(name)
            inner = QtWidgets.QVBoxLayout(box)
            value = QtWidgets.QLabel('—')
            value.setStyleSheet(
                'color:#67e8f9;font-size:18px;font-weight:bold')
            inner.addWidget(value)
            self.metrics[name] = value
            metrics_row.addWidget(box)
        root.addLayout(metrics_row)
        self.tabs = QtWidgets.QTabWidget()
        root.addWidget(self.tabs, 1)
        panel = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(panel)
        self.accplot = self.plot('Acceleration | time', 'g', 's')
        self.psdplot = self.plot('Welch PSD | acceleration', 'g²/Hz', 'Hz')
        self.gyroplot = self.plot('Gyroscope | time', '°/s', 's')
        self.magplot = self.plot('Acceleration magnitude | time', 'g', 's')
        for widget, r, c in [(self.accplot, 0, 0), (self.psdplot, 0, 1), (self.gyroplot, 1, 0), (self.magplot, 1, 1)]:
            grid.addWidget(widget, r, c)
        self.tabs.addTab(panel, 'Live analysis')
        self.airplane = Aircraft()
        self.tabs.addTab(self.airplane, '3D Rafale')
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.tabs.addTab(self.log, 'Diagnostics')
        colors = ['#38bdf8', '#34d399', '#fbbf24']
        self.acc_curves = [self.accplot.plot(
            pen=pg.mkPen(c, width=1.4)) for c in colors]
        self.gyro_curves = [self.gyroplot.plot(
            pen=pg.mkPen(c, width=1.4)) for c in colors]
        self.psd_curve = self.psdplot.plot(pen=pg.mkPen('#38bdf8', width=1.6))
        self.mag_curve = self.magplot.plot(pen=pg.mkPen('#a78bfa', width=1.6))
        bottom = QtWidgets.QHBoxLayout()
        self.pause_btn = QtWidgets.QPushButton('Pause display')
        self.pause_btn.clicked.connect(self.toggle_pause)
        self.record_btn = QtWidgets.QPushButton('Start CSV')
        self.record_btn.clicked.connect(self.toggle_record)
        clear = QtWidgets.QPushButton('Clear history')
        clear.clicked.connect(lambda: self.reset_data(True))
        export = QtWidgets.QPushButton('Save PNG')
        export.clicked.connect(self.export_png)
        self.window = self.combo(['5', '10', '20', '60'], '10')
        self.channel = self.combo(['X', 'Y', 'Z', 'Magnitude'], 'X')
        for w in [self.pause_btn, self.record_btn, clear, export, QtWidgets.QLabel('Window s'), self.window, QtWidgets.QLabel('PSD channel'), self.channel]:
            bottom.addWidget(w)
        root.addLayout(bottom)
        self.footer = QtWidgets.QLabel(
            f'Firmware v2 required | 460800 baud | Sensor forward {args.sensor_forward}, '
            f'up {args.sensor_up} | 8-field CSV + CFG g hz dlpf gyro')
        root.addWidget(self.footer)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    @staticmethod
    def combo(items, current, editable=False):
        w = QtWidgets.QComboBox()
        w.setEditable(editable)
        w.addItems(items)
        w.setCurrentText(current)
        return w

    @staticmethod
    def plot(title, yunit, xunit):
        w = pg.PlotWidget(title=title)
        w.setLabel('left', units=yunit)
        w.setLabel('bottom', units=xunit)
        w.showGrid(x=True, y=True, alpha=.2)
        return w

    def log_event(self, message):
        self.log.appendPlainText(time.strftime(
            '%H:%M:%S') + '  ' + str(message))

    def reset_data(self, announce=False):
        self.T = deque(maxlen=65000)
        self.A = [deque(maxlen=65000) for _ in range(3)]
        self.G = [deque(maxlen=65000) for _ in range(3)]
        self.dt = deque(maxlen=4000)
        self.lastseq = self.lastt = None
        self.lost = self.clips = self.gyroclips = self.invalid = 0
        if announce and hasattr(self, 'log'):
            self.log_event('History reset; CSV recording continues')

    def reset_attitude(self):
        self.estimator.reset()
        self.roll = self.pitch = self.yaw = 0.0
        self.orientation = R.identity()
        self.orientation_stationary = False
        self.orientation_status = 'Hold sensor still to calibrate gyro bias'

    def toggle_connection(self):
        if self.ser:
            self.disconnect()
            return
        try:
            self.ser = serial.Serial(
                self.port.currentText().strip(), args.baud, timeout=0)
            self.ser.reset_input_buffer()
            self.partial = b''
            self.reset_data()
            self.reset_attitude()
            self.connection.setText('Disconnect')
            self.status.setText('CONNECTED / WAITING')
            self.log_event(f'Connected {self.ser.port} at {args.baud} baud')
            self.apply_config()
        except Exception as exc:
            self.ser = None
            QtWidgets.QMessageBox.warning(self, 'Connection error', str(exc))

    def disconnect(self):
        if self.ser:
            self.ser.close()
        self.ser = None
        self.connection.setText('Connect')
        self.status.setText('DISCONNECTED')
        self.log_event('Disconnected')

    def apply_config(self):
        if not self.ser:
            return
        config = (int(self.g.currentText()), int(self.hz.currentText()),
                  self.filt.currentIndex(), int(self.gyro.currentText()))
        self.ser.write(('CFG %d %d %d %d\n' % config).encode())
        self.pending = (config, time.monotonic())
        self.log_event(f'Awaiting configuration ACK: {config}')

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.setText(
            'Resume display' if self.paused else 'Pause display')

    def toggle_record(self):
        if self.file:
            self.file.close()
            self.file = self.writer = None
            self.record_btn.setText('Start CSV')
            return
        if not self.ser:
            QtWidgets.QMessageBox.information(
                self, 'Recording', 'Connect first')
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, 'Save CSV', time.strftime('imu_%Y%m%d_%H%M%S.csv'), 'CSV (*.csv)')
        if path:
            self.file = open(path, 'w', newline='', encoding='utf-8')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['seq', 't_us', 'accel_range_g', 'rate_hz', 'dlpf',
                                 'gyro_range_dps', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'])
            self.record_btn.setText('Stop CSV')

    def export_png(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, 'Export acceleration plot', 'acceleration.png', 'PNG (*.png)')
        if path:
            ImageExporter(self.accplot.plotItem).export(path)

    def parse_line(self, line):
        if line.startswith('#'):
            self.log_event(line)
            if line.startswith('# CONFIG '):
                parts = line.split()
                if len(parts) == 7 and parts[-1] == 'OK':
                    try:
                        g, hz, filt, gyro = map(int, parts[2:6])
                    except ValueError:
                        return
                    self.range, self.rate, self.dlpf, self.gyro_range = g, hz, filt, gyro
                    self.pending = None
                    self.reset_data()
                    self.reset_attitude()
                    self.g.setCurrentText(str(g))
                    self.hz.setCurrentText(str(hz))
                    self.filt.setCurrentIndex(filt)
                    self.gyro.setCurrentText(str(gyro))
            return
        parts = line.split(',')
        if len(parts) != 8:
            self.invalid += 1
            return
        try:
            seq, t, ax, ay, az, gx, gy, gz = map(int, parts)
        except ValueError:
            self.invalid += 1
            return
        if self.lastseq is not None:
            if seq > self.lastseq + 1:
                self.lost += seq - self.lastseq - 1
            elif seq <= self.lastseq:
                self.log_event('Sequence reset; clearing counters')
                self.reset_data()
                self.reset_attitude()
        self.lastseq = seq
        if self.lastt is not None:
            dt = (t - self.lastt) & 0xffffffff
            if 0 < dt < 100000:
                self.dt.append(dt)
        self.lastt = t
        asc = {2: 16384., 4: 8192., 8: 4096., 16: 2048.}[self.range]
        gsc = {250: 131., 500: 65.5, 1000: 32.8, 2000: 16.4}[self.gyro_range]
        a = np.array([ax, ay, az], dtype=float) / asc
        w = np.array([gx, gy, gz], dtype=float) / gsc
        bias_was_ready = self.estimator.bias_ready
        attitude = self.estimator.update(t, a, w)
        self.orientation = attitude.orientation
        self.orientation_stationary = attitude.stationary
        self.orientation_status = attitude.status
        self.roll = attitude.roll_deg
        self.pitch = attitude.pitch_deg
        self.yaw = attitude.yaw_deg
        if attitude.bias_ready and not bias_was_ready:
            bx, by, bz = attitude.gyro_bias_dps
            self.log_event(
                f'Gyro bias calibrated: X {bx:+.4f}, Y {by:+.4f}, Z {bz:+.4f} °/s')
        self.T.append(t / 1e6)
        for buf, value in zip(self.A, a):
            buf.append(value)
        for buf, value in zip(self.G, w):
            buf.append(value)
        if max(abs(ax), abs(ay), abs(az)) >= 32000:
            self.clips += 1
        if max(abs(gx), abs(gy), abs(gz)) >= 32000:
            self.gyroclips += 1
        if self.writer:
            self.writer.writerow(
                [seq, t, self.range, self.rate, self.dlpf, self.gyro_range, *a, *w])

    def tick(self):
        if not self.ser:
            return
        try:
            # Preserve partial lines between GUI updates.
            available = min(self.ser.in_waiting, 250000)
            if available:
                self.partial += self.ser.read(available)
                lines = self.partial.split(b'\n')
                self.partial = lines.pop()
                if len(self.partial) > 4096:
                    self.partial = b''
                    self.invalid += 1
                for line in lines:
                    self.parse_line(line.decode(
                        'ascii', errors='replace').strip())
        except (serial.SerialException, OSError) as exc:
            self.log_event(f'Serial error: {exc}')
            self.disconnect()
            return
        if self.pending and time.monotonic() - self.pending[1] > 4:
            self.log_event(
                'Configuration ACK timeout; check matching firmware')
            self.pending = None
        if len(self.T) < 20:
            return
        self.status.setText('STREAMING / DATA VALID')
        if self.paused:
            return
        if self.tabs.currentWidget() is self.airplane:
            self.airplane.set_orientation(
                self.orientation,
                self.estimator.sensor_to_aircraft,
                self.orientation_stationary,
                self.orientation_status,
            )
        tt = np.asarray(self.T)
        index = np.searchsorted(tt, tt[-1] - float(self.window.currentText()))
        t = tt[index:] - tt[-1]
        aa = [np.asarray(v)[index:] for v in self.A]
        gg = [np.asarray(v)[index:] for v in self.G]
        step = max(1, len(t) // 2500)
        for curve, values in zip(self.acc_curves, aa):
            curve.setData(t[::step], values[::step])
        for curve, values in zip(self.gyro_curves, gg):
            curve.setData(t[::step], values[::step])
        mag = np.sqrt(sum(v*v for v in aa))
        self.mag_curve.setData(t[::step], mag[::step])
        fs = 1e6 / np.mean(self.dt) if len(self.dt) > 20 else float(self.rate)
        signal = (aa + [mag])[self.channel.currentIndex()]
        if len(signal) >= 128:
            s = signal[-min(len(signal), 8192):]
            f, power = welch(s-np.mean(s), fs=fs, nperseg=min(2048, len(s)))
            self.psd_curve.setData(f, power)
        values = {'Read Hz': f'{fs:.1f} Hz', 'Jitter SD': f'{np.std(self.dt):.1f} µs' if self.dt else '—',
                  'Sequence gaps': str(self.lost), 'Accel RMS X': f'{np.std(aa[0])*1000:.1f} mg',
                  'Peak |a|': f'{np.max(mag):.2f} g', 'Accel clips': str(self.clips), 'Gyro clips': str(self.gyroclips)}
        for name, value in values.items():
            self.metrics[name].setText(value)
        self.footer.setText(
            f'Confirmed: ±{self.range}g / {self.rate}Hz / DLPF {self.dlpf} / Gyro ±{self.gyro_range}°/s | Malformed: {self.invalid} | Read cadence ≠ independently verified sensor rate')

    def closeEvent(self, event):
        if self.file:
            self.file.close()
        if self.ser:
            self.ser.close()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    app.setStyleSheet('QWidget{background:#0b1220;color:#e2e8f0;font-size:12px} QPushButton,QComboBox{background:#1b2a41;border:1px solid #33465e;border-radius:5px;padding:6px} QPushButton:hover{background:#28415f} QGroupBox{border:1px solid #33465e;border-radius:6px;margin-top:10px;padding:8px} QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px}')
    lab = Lab()
    lab.show()
    app.exec()
