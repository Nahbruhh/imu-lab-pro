# Serial protocol

The host configures the device with:

```text
CFG <accel_range_g> <rate_hz> <dlpf> <gyro_range_dps>
```

The device emits raw signed sensor counts:

```text
seq,t_us,ax,ay,az,gx,gy,gz
```

Lines beginning with `#` are status or configuration messages. Sequence gaps
currently identify missing serial records; they do not independently prove
missing MPU6050 internal samples. Firmware health counters and independently
verified data-ready/FIFO acquisition remain future work.
