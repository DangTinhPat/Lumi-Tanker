# GDS68 + USB-CAN-A Bring-up

Tai lieu nay tong hop qua trinh test thanh cong dong co GDS68/GIM6010-8 bang
adapter Waveshare USB-CAN-A tren Linux.

## 1. Phan cung da test

- Dong co/driver: GDS68, firmware dua tren ODrive/CAN Simple.
- Adapter CAN: Waveshare USB-CAN-A, Linux nhan dang la CH340 serial.
- Cong dung dung cho GUI: `/dev/ttyUSB0`.
- Baud USB serial adapter: `2000000`.
- CAN baudrate da test thanh cong: `500 kbps`.
- Node dang test thanh cong: `node_id = 1`.

Luu y: tren may cung co the thay `/dev/ttyACM0`, nhung day la cong USB truc
tiep cua driver/ODrive, khong phai USB-CAN-A. Khi dung chuong trinh
`gds68_controller.py`, phai chon `/dev/ttyUSB0`.

## 2. Kiem tra thiet bi

Kiem tra USB:

```bash
lsusb
```

Ket qua can thay USB-CAN-A dang la CH340:

```text
1a86:7523 QinHeng Electronics CH340 serial converter
```

Kiem tra cong serial:

```bash
python3 - <<'PY'
import serial.tools.list_ports
for p in serial.tools.list_ports.comports():
    print(p.device, p.description, p.hwid)
PY
```

Ket qua quan trong:

```text
/dev/ttyUSB0 USB Serial USB VID:PID=1A86:7523
```

Kiem tra quyen truy cap:

```bash
id
ls -l /dev/ttyUSB0
```

User can nam trong group `dialout`.

## 3. Loi da gap va cach sua

### 3.1 Chon nham cong serial

GUI ban dau hien nhieu cong `/dev/ttyS*`, de chon nham. Da sua de uu tien
cong USB serial nhu:

- `/dev/ttyUSB0`
- `/dev/ttyACM0`

Khi test voi USB-CAN-A, chon `/dev/ttyUSB0`.

### 3.2 Parser RX cua Waveshare sai do dai frame

Frame serial cua Waveshare USB-CAN-A dang dung:

```text
AA TYPE ID_LO ID_HI DATA... 55
```

Voi standard CAN frame va DLC=8, tong do dai la 13 byte:

```text
1 header + 1 type + 2 id + 8 data + 1 tail = 13
```

Code cu tinh sai thanh 14 byte nen GUI co the khong parse duoc RX. Da sua
trong `gds68_controller.py`:

```python
expected_len = (7 + dlc) if extended else (5 + dlc)
```

Sau khi sua, may doc duoc frame CAN tu driver:

```text
ID=0x021 data=00 00 00 00 01 80 26 FC
ID=0x029 data=00 00 00 00 00 00 00 00
```

`0x021` la heartbeat cua node 1:

```text
CAN ID = (node_id << 5) | cmd_id
0x021  = (1 << 5) | 0x001
```

Dieu nay xac nhan bus CAN, USB-CAN-A, baudrate va node 1 da hoat dong.

## 4. Chay GUI

Chay chuong trinh:

```bash
cd /home/dvt/TEST1
python3 gds68_controller.py
```

Neu muon mo GUI tu terminal va de no chay nen:

```bash
setsid python3 /home/dvt/TEST1/gds68_controller.py >/tmp/gds68_controller.log 2>&1 &
```

Trong GUI chon:

```text
Port: /dev/ttyUSB0
CAN Baudrate: 500 kbps
Joint / Node ID: LeftHip / ID=1
```

Sau do bam:

```text
Ket noi USB-CAN-A
```

Neu ket noi dung, log CAN se thay RX heartbeat hoac encoder estimate, vi du:

```text
ID=0x021
ID=0x029
```

## 5. Cho dong co quay cham

GUI da duoc them che do quay cham:

- `Quay cham -`
- `Dung quay cham`
- `Quay cham +`
- Thanh `Toc do quay cham (rad/s)`

Gia tri mac dinh:

```text
Toc do: 0.15 rad/s
Kp:     8.0
Kd:     0.6
Chu ky gui lenh: 50 ms
```

`0.15 rad/s` tuong duong khoang `1.4 rpm` o truc ra.

Quy trinh test:

1. Dat dong co o vi tri an toan, khong bi kep co khi.
2. Bat nguon driver.
3. Mo GUI.
4. Chon `/dev/ttyUSB0`, `500 kbps`, `ID=1`.
5. Bam `Ket noi USB-CAN-A`.
6. Bam `Quay cham +` hoac `Quay cham -`.
7. Bam `Dung quay cham` de dung lenh ramp.
8. Bam `Disable (Idle)` de dua driver ve idle.

Che do quay cham dung MIT position ramp: chuong trinh tang/giam target
position tung buoc nho theo thoi gian. Khi target gan gioi han MIT
`-12.5 .. 12.5 rad`, chuong trinh tu dung ramp de tranh vuot gioi han.

## 6. Frame CAN quan trong

Cong thuc CAN ID:

```text
CAN ID = (node_id << 5) | cmd_id
```

Voi `node_id = 1`:

```text
Set_Controller_Mode: ID=0x02B
Set_Axis_State:      ID=0x027
Mit_Control:         ID=0x028
Heartbeat:           ID=0x021
Encoder Estimates:   ID=0x029
```

Thu tu enable MIT:

```text
1. ID=0x02B Data=03 00 00 00 09 00 00 00
2. ID=0x027 Data=08 00 00 00 00 00 00 00
3. ID=0x028 Data=<MIT control frame>
```

## 7. Neu ket noi duoc nhung dong co khong quay

Kiem tra cac muc sau:

1. Node ID co dung khong. Hien tai test thanh cong voi `node_id = 1`.
2. CAN baudrate co dung `500 kbps` khong.
3. Driver da calibration motor va encoder chua.
4. Driver co vao closed-loop duoc khong.
5. Co loi axis/motor/encoder khong, kiem tra bang MotorWizard hoac
   `odrivetool`.
6. Day CAN-H/CAN-L co dau dung khong va bus co dien tro ket thuc phu hop
   khong.

Neu da thay heartbeat `ID=0x021`, phan USB-CAN-A va bus CAN da co tin hieu.
Khi do loi thuong nam o calibration, closed-loop, node ID, mode dieu khien
hoac gioi han co khi.

## 8. File lien quan

- `gds68_controller.py`: GUI dieu khien USB-CAN-A, MIT control, quay cham.
- `GDS68_DRIVER.md`: tai lieu protocol GDS68/CAN Simple chi tiet.
- `Document-of-GDS68-Driver.zip`: tai lieu goc cua nha san xuat.

