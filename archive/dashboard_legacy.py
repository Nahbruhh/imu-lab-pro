import argparse, csv, time
from collections import deque
import numpy as np
import serial
from serial.tools import list_ports
from scipy.signal import welch
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter

p=argparse.ArgumentParser();p.add_argument('--port',default='COM4');p.add_argument('--baud',type=int,default=460800);args=p.parse_args()
app=QtWidgets.QApplication([]);app.setStyle('Fusion')
app.setStyleSheet('''QWidget{background:#0b1220;color:#e2e8f0;font-size:12px}QPushButton,QComboBox{background:#1b2a41;border:1px solid #33465e;border-radius:6px;padding:7px}QPushButton:hover{background:#28415f}QLabel#title{font-size:23px;font-weight:bold}QGroupBox{border:1px solid #2b3c54;border-radius:8px;margin-top:12px;padding:12px}QGroupBox::title{subcontrol-origin:margin;left:12px;padding:0 5px}''')
pg.setConfigOptions(antialias=False,background='#111c2e',foreground='#cbd5e1')

class Lab(QtWidgets.QWidget):
 def __init__(self):
  super().__init__();self.setWindowTitle('IMU LAB PRO v2 | ESP32-S3 + MPU6050');self.resize(1550,920)
  self.ser=None;self.record_file=None;self.writer=None;self.paused=False;self.pending=None
  self.range=2;self.rate=200;self.dlpf=3;self.gyro_range=250;self.reset_data(False)
  root=QtWidgets.QVBoxLayout(self)
  top=QtWidgets.QHBoxLayout();title=QtWidgets.QLabel('IMU LAB / PRO v2');title.setObjectName('title');top.addWidget(title);top.addStretch();self.status=QtWidgets.QLabel('● DISCONNECTED');top.addWidget(self.status);root.addLayout(top)
  controls=QtWidgets.QGroupBox('DEVICE & ACQUISITION');bar=QtWidgets.QHBoxLayout(controls)
  self.port=QtWidgets.QComboBox();self.port.setEditable(True);self.port.addItems([x.device for x in list_ports.comports()]);self.port.setCurrentText(args.port)
  self.connect_btn=QtWidgets.QPushButton('Connect');self.connect_btn.clicked.connect(self.toggle_connection)
  self.g=self.combo(['2','4','8','16'],'2');self.hz=self.combo(['50','100','200','500','1000'],'200')
  self.filt=self.combo(['0 · 260 Hz','1 · 184 Hz','2 · 94 Hz','3 · 44 Hz','4 · 21 Hz','5 · 10 Hz','6 · 5 Hz'],'3 · 44 Hz')
  self.gyro=self.combo(['250','500','1000','2000'],'250')
  self.apply_btn=QtWidgets.QPushButton('Apply settings');self.apply_btn.clicked.connect(self.apply_config)
  for label,w in [('Port',self.port),('',self.connect_btn),('Accel ±g',self.g),('Rate Hz',self.hz),('DLPF',self.filt),('Gyro ±°/s',self.gyro),('',self.apply_btn)]:
   if label:bar.addWidget(QtWidgets.QLabel(label))
   bar.addWidget(w)
  root.addWidget(controls)
  cards=QtWidgets.QHBoxLayout();self.metrics={}
  for name in ['Read Hz','Jitter SD','Sequence gaps','Accel RMS X','Peak |a|','Accel clips','Gyro clips']:
   box=QtWidgets.QGroupBox(name);l=QtWidgets.QVBoxLayout(box);v=QtWidgets.QLabel('—');v.setStyleSheet('font-size:19px;font-weight:bold;color:#67e8f9');l.addWidget(v);cards.addWidget(box);self.metrics[name]=v
  root.addLayout(cards)
  self.tabs=QtWidgets.QTabWidget();root.addWidget(self.tabs,1)
  panel=QtWidgets.QWidget();grid=QtWidgets.QGridLayout(panel)
  self.accplot=self.make_plot('Acceleration | time','Acceleration','g','Time','s')
  self.psdplot=self.make_plot('Welch PSD | acceleration','PSD','g²/Hz','Frequency','Hz')
  self.gyroplot=self.make_plot('Gyroscope | time','Angular rate','°/s','Time','s')
  self.magplot=self.make_plot('Acceleration magnitude | time','Magnitude','g','Time','s')
  for plot,r,c in [(self.accplot,0,0),(self.psdplot,0,1),(self.gyroplot,1,0),(self.magplot,1,1)]:grid.addWidget(plot,r,c)
  self.tabs.addTab(panel,'Live analysis');self.log=QtWidgets.QPlainTextEdit();self.log.setReadOnly(True);self.tabs.addTab(self.log,'Diagnostics')
  colors=['#38bdf8','#34d399','#fbbf24'];self.acc_curves=[self.accplot.plot(pen=pg.mkPen(c,width=1.5)) for c in colors]
  self.gyro_curves=[self.gyroplot.plot(pen=pg.mkPen(c,width=1.5)) for c in colors];self.psd_curve=self.psdplot.plot(pen=pg.mkPen('#38bdf8',width=1.6));self.mag_curve=self.magplot.plot(pen=pg.mkPen('#a78bfa',width=1.6))
  lower=QtWidgets.QHBoxLayout();self.pause_btn=QtWidgets.QPushButton('Pause display');self.pause_btn.clicked.connect(self.toggle_pause)
  self.record_btn=QtWidgets.QPushButton('Start CSV');self.record_btn.clicked.connect(self.toggle_record)
  self.clear_btn=QtWidgets.QPushButton('Clear history');self.clear_btn.clicked.connect(lambda:self.reset_data(True))
  self.export_btn=QtWidgets.QPushButton('Save PNG');self.export_btn.clicked.connect(self.export_png)
  self.window=self.combo(['5','10','20','60'],'10');self.channel=self.combo(['X','Y','Z','Magnitude'],'X')
  for w in [self.pause_btn,self.record_btn,self.clear_btn,self.export_btn,QtWidgets.QLabel('Window s'),self.window,QtWidgets.QLabel('PSD channel'),self.channel]:lower.addWidget(w)
  root.addLayout(lower);self.footer=QtWidgets.QLabel('Firmware v2 required | 460800 baud | Start at ±2g / 200Hz / DLPF 3 / ±250°/s');root.addWidget(self.footer)
  self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(50);app.aboutToQuit.connect(self.shutdown)
 def combo(self,items,selected):
  w=QtWidgets.QComboBox();w.addItems(items);w.setCurrentText(selected);return w
 def make_plot(self,title,yl,yu,xl,xu):
  w=pg.PlotWidget(title=title);w.setLabel('left',yl,units=yu);w.setLabel('bottom',xl,units=xu);w.showGrid(x=True,y=True,alpha=.2);return w
 def log_event(self,msg):
  if hasattr(self,'log'):self.log.appendPlainText(time.strftime('%H:%M:%S')+' '+str(msg))
 def reset_data(self,announce=True):
  self.T=deque(maxlen=65000);self.A=[deque(maxlen=65000) for _ in range(3)];self.G=[deque(maxlen=65000) for _ in range(3)];self.dt=deque(maxlen=4000)
  self.lastseq=None;self.lastt=None;self.lost=0;self.clips=0;self.gyroclips=0;self.invalid=0
  if announce:self.log_event('Plot history and diagnostics reset (recording continues)')
 def toggle_connection(self):
  if self.ser:self.disconnect();return
  try:
   self.ser=serial.Serial(self.port.currentText().strip(),args.baud,timeout=0);self.ser.reset_input_buffer();self.reset_data(False)
   self.connect_btn.setText('Disconnect');self.status.setText('● CONNECTED / WAITING FOR DATA');self.status.setStyleSheet('color:#fbbf24');self.log_event('Connected '+self.ser.port+' at '+str(args.baud)+' baud')
   self.apply_config()
  except Exception as e:self.ser=None;QtWidgets.QMessageBox.warning(self,'Connection error',str(e))
 def disconnect(self):
  if self.ser:
   try:self.ser.close()
   except Exception:pass
  self.ser=None;self.connect_btn.setText('Connect');self.status.setText('● DISCONNECTED');self.status.setStyleSheet('color:#f87171');self.log_event('Disconnected')
 def apply_config(self):
  if not self.ser:return
  g=int(self.g.currentText());hz=int(self.hz.currentText());d=self.filt.currentIndex();gyro=int(self.gyro.currentText())
  try:
   self.ser.write(f'CFG {g} {hz} {d} {gyro}\n'.encode());self.pending=(g,hz,d,gyro,time.monotonic());self.log_event(f'Sent CFG {g} {hz} {d} {gyro}; awaiting ACK')
  except Exception as e:self.log_event('Send failed: '+str(e))
 def toggle_pause(self):
  self.paused=not self.paused;self.pause_btn.setText('Resume display' if self.paused else 'Pause display');self.log_event('Display paused; acquisition continues' if self.paused else 'Display resumed')
 def toggle_record(self):
  if self.record_file:
   self.record_file.close();self.record_file=None;self.writer=None;self.record_btn.setText('Start CSV');self.log_event('CSV stopped');return
  if not self.ser:QtWidgets.QMessageBox.information(self,'Recording','Connect first');return
  path,_=QtWidgets.QFileDialog.getSaveFileName(self,'Save measurement',time.strftime('imu_%Y%m%d_%H%M%S.csv'),'CSV (*.csv)')
  if not path:return
  self.record_file=open(path,'w',newline='',encoding='utf-8');self.writer=csv.writer(self.record_file)
  self.writer.writerow(['seq','t_us','accel_range_g','rate_hz','dlpf','gyro_range_dps','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps']);self.record_btn.setText('Stop CSV');self.log_event('Recording '+path)
 def export_png(self):
  path,_=QtWidgets.QFileDialog.getSaveFileName(self,'Export plot','acceleration.png','PNG (*.png)')
  if path:
   try:ImageExporter(self.accplot.plotItem).export(path);self.log_event('Saved '+path)
   except Exception as e:self.log_event('Export failed '+str(e))
 def parse_line(self,s):
  if s.startswith('#'):
   self.log_event(s)
   if s.startswith('# CONFIG '):
    parts=s.split()
    if len(parts)==7 and parts[-1]=='OK':
     try:g,hz,d,gyro=map(int,parts[2:6])
     except ValueError:return
     self.range,self.rate,self.dlpf,self.gyro_range=g,hz,d,gyro;self.pending=None;self.reset_data(False)
     self.g.setCurrentText(str(g));self.hz.setCurrentText(str(hz));self.filt.setCurrentIndex(d);self.gyro.setCurrentText(str(gyro))
     self.log_event(f'Confirmed ±{g}g / {hz}Hz / DLPF {d} / gyro ±{gyro}°/s; counters reset')
   return
  parts=s.split(',')
  if len(parts)!=8:self.invalid+=1;return
  try:seq,t,ax,ay,az,gx,gy,gz=map(int,parts)
  except ValueError:self.invalid+=1;return
  if self.lastseq is not None:
   if seq>self.lastseq+1:self.lost+=seq-self.lastseq-1
   elif seq<=self.lastseq:self.log_event('Sequence reset/out-of-order; restarting counters');self.reset_data(False)
  self.lastseq=seq
  if self.lastt is not None:
   dt=(t-self.lastt)&0xffffffff
   if 0<dt<100000:self.dt.append(dt)
  self.lastt=t
  accel_scale={2:16384,4:8192,8:4096,16:2048}[self.range]
  gyro_scale={250:131.,500:65.5,1000:32.8,2000:16.4}[self.gyro_range]
  self.T.append(t/1e6)
  for buf,val in zip(self.A,[ax,ay,az]):buf.append(val/accel_scale)
  for buf,val in zip(self.G,[gx,gy,gz]):buf.append(val/gyro_scale)
  if max(abs(ax),abs(ay),abs(az))>=32000:self.clips+=1
  if max(abs(gx),abs(gy),abs(gz))>=32000:self.gyroclips+=1
  if self.writer:self.writer.writerow([seq,t,self.range,self.rate,self.dlpf,self.gyro_range,ax/accel_scale,ay/accel_scale,az/accel_scale,gx/gyro_scale,gy/gyro_scale,gz/gyro_scale])
 def tick(self):
  if not self.ser:return
  try:
   # Read complete lines only. Keep partial bytes in the serial driver buffer.
   for _ in range(2500):
    if self.ser.in_waiting<1:break
    raw=self.ser.readline()
    if not raw:break
    if not raw.endswith(b'\n'):
     # Keep incomplete line for next GUI tick.
     self.partial=getattr(self,'partial',b'')+raw
     if len(self.partial)>1024:self.partial=b''
     break
    raw=getattr(self,'partial',b'')+raw;self.partial=b''
    self.parse_line(raw.decode('ascii',errors='replace').strip())
  except (serial.SerialException,OSError) as e:self.log_event('Serial failure '+str(e));self.disconnect();return
  if self.pending and time.monotonic()-self.pending[4]>4:
   self.log_event('No config acknowledgement; check firmware v2 and baud');self.pending=None
  if len(self.T)<20:return
  self.status.setText('● STREAMING / DATA VALID');self.status.setStyleSheet('color:#34d399')
  if self.paused:return
  tt=np.asarray(self.T,dtype=float);window=float(self.window.currentText());idx=np.searchsorted(tt,tt[-1]-window)
  t=tt[idx:]-tt[-1];aa=[np.asarray(x,dtype=float)[idx:] for x in self.A];gg=[np.asarray(x,dtype=float)[idx:] for x in self.G]
  # Plot decimation affects display only, not acquisition or CSV recording.
  step=max(1,len(t)//2500)
  for curve,v in zip(self.acc_curves,aa):curve.setData(t[::step],v[::step])
  for curve,v in zip(self.gyro_curves,gg):curve.setData(t[::step],v[::step])
  mag=np.sqrt(aa[0]**2+aa[1]**2+aa[2]**2);self.mag_curve.setData(t[::step],mag[::step])
  fs=1e6/np.mean(self.dt) if len(self.dt)>20 else float(self.rate)
  signal=(aa+[mag])[self.channel.currentIndex()]
  if len(signal)>=128:
   s=signal[-min(8192,len(signal)):];f,power=welch(s-np.mean(s),fs=fs,nperseg=min(2048,len(s)));self.psd_curve.setData(f,power)
  self.metrics['Read Hz'].setText(f'{fs:.1f} Hz')
  self.metrics['Jitter SD'].setText(f'{np.std(self.dt):.1f} µs' if self.dt else '—')
  self.metrics['Sequence gaps'].setText(str(self.lost))
  self.metrics['Accel RMS X'].setText(f'{np.std(aa[0])*1000:.1f} mg')
  self.metrics['Peak |a|'].setText(f'{np.max(mag):.2f} g')
  self.metrics['Accel clips'].setText(str(self.clips))
  self.metrics['Gyro clips'].setText(str(self.gyroclips))
  self.footer.setText(f'Confirmed: ±{self.range}g | {self.rate}Hz | DLPF {self.dlpf} | Gyro ±{self.gyro_range}°/s | Malformed lines: {self.invalid} | Read rate is not independent sensor-data-ready rate')
 def shutdown(self):
  if self.record_file:self.record_file.close()
  if self.ser:self.ser.close()

lab=Lab();lab.show();app.exec()
