# Scheerenrobot – Bedienungsanleitung

Referenz für den laufenden Betrieb: welche ROS2-Topics es gibt, wie man sie
benutzt. Abgrenzung zu den anderen Doku-Dateien:

- `ROBOT_PLAN.md` – Architektur, Mission, Hardware-Planung, Phasen-Fortschritt
- `idf-micro-ros-setup.md` – ESP-IDF-Build-/Flash-Referenz für die Firmware-Entwicklung
- **diese Datei** – wie man den fertig geflashten Roboter über ROS2 *bedient*

Wächst mit dem Projekt: neue Topics/Features kommen als eigene Abschnitte dazu.

---

## Voraussetzungen

- Beide micro-ROS-Agent-Container laufen (siehe `startbefehl.txt`):
  `microros-agent-robot` (`/dev/ttyUSB-robot`), `microros-agent-lift` (`/dev/ttyUSB-lift`)
- `ros2dev`-Container läuft, darin `source /ros2_ws/install/setup.bash`

---

## Robot-Board (`ybMecanumWheelMicroRosBot`, Node `/YB_Car_Node`)

### Antrieb

| Topic | Typ | Richtung | Beschreibung |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Fahrbefehl (linear.x/y, angular.z) |
| `/odom_raw` | `nav_msgs/Odometry` | Pub, 11Hz | Odometrie aus Encodern |
| `/imu` | `sensor_msgs/Imu` | Pub, 25Hz | ICM42670P Rohdaten |

### Kalibrierung (`/calibrate`)

`geometry_msgs/Vector3` — `x`=Raddurchmesser (mm), `y`=Spurbreite (m), `z`=Radstand (m).
Wird sofort angewendet und ins NVS persistiert (Namespace `calib`).

```bash
ros2 topic pub --once /calibrate geometry_msgs/msg/Vector3 "{x: 64.0, y: 0.152, z: 0.105}"
```

### Signalton (`/beep`)

`std_msgs/UInt16`:

| Wert | Verhalten |
|---|---|
| `0` | Aus |
| `1` | Dauerton |
| `10`–`10000` | Impuls (ms), schaltet danach automatisch ab |

```bash
ros2 topic pub --once /beep std_msgs/msg/UInt16 "{data: 300}"
```

### Batterie

**`/battery_state`** (`sensor_msgs/BatteryState`, Pub, 1Hz) — Nur-Lesen, Live-Messwerte:

| Feld | Bedeutung |
|---|---|
| `voltage` | Pack-Spannung in V (per ADC GPIO3, kalibrierter Spannungsteiler) |
| `percentage` | 0.0–1.0, linear zwischen Cutoff- und Max-Spannung |
| `power_supply_health` | `GOOD` (1) oder `DEAD` (3), abhängig vom Cutoff |
| `power_supply_technology` | `LION` (2) oder `LIPO` (3) |
| `design_capacity` | Konfigurierte Kapazität in Ah |
| `temperature`, `current`, `charge`, `capacity` | `NaN` — keine Sensorik dafür vorhanden |

```bash
ros2 topic echo /battery_state
```

**`/battery_config`** (`std_msgs/Int32MultiArray`, Sub) — Pack-Konfiguration
setzen. Wird sofort wirksam **und** ins NVS persistiert (Namespace `battery`,
übersteht Reboot/Reset). Erwartet **genau 7 Werte in fester Reihenfolge**,
sonst wird die Nachricht verworfen (Fehlerlog, kein Crash):

| Index | Feld | Einheit | Kconfig-Default | Bedeutung |
|---|---|---|---|---|
| 0 | `cell_count` | Anzahl Zellen | `2` | Zellen in Serie (2S = 7,4V nominal) |
| 1 | `capacity_mah` | mAh | `2000` | Nennkapazität des Packs |
| 2 | `cell_voltage_max_mv` | mV/Zelle | `4200` | Vollgeladen-Spannung pro Zelle |
| 3 | `cell_voltage_warn_mv` | mV/Zelle | `3450` | Schwelle für periodischen Warnton (Buzzer) |
| 4 | `cell_voltage_cutoff_mv` | mV/Zelle | `3450` | Schwelle für `power_supply_health=DEAD` |
| 5 | `technology` | Enum | `3` | `2`=Li-Ion, `3`=LiPo (`sensor_msgs/BatteryState`-Konstante) |
| 6 | `adc_divider_factor_x1000` | Faktor×1000 | `4078` | Spannungsteiler-Kalibrierung, per Multimeter ermittelt |

```bash
# Beispiel: 2S, 2000mAh, Warnschwelle 3.6V/Zelle (7.2V Pack), Cutoff 3.45V/Zelle (6.9V Pack), LiPo
ros2 topic pub --once /battery_config std_msgs/msg/Int32MultiArray \
  "{data: [2, 2000, 4200, 3600, 3450, 3, 4078]}"
```

**Wichtig:** Alle Spannungswerte außer `cell_voltage_max/warn/cutoff_mv` sind
Pack-weite Größen, die Spannungswerte selbst sind **pro Zelle** — bei 2S also
mit 2 multiplizieren für die tatsächliche Pack-Spannung (z.B. `warn_mv=3600`
→ Warnung unter 7,2V Pack-Spannung).

**Warnton-Verhalten:** Solange `voltage < cell_count × cell_voltage_warn_mv`,
piept der Buzzer automatisch alle ~1s kurz (150ms) — kein manuelles Zutun
nötig, stoppt von selbst sobald die Spannung wieder über der Schwelle liegt.

**Cutoff-Default-Begründung:** Die Zellchemie des verbauten Akkus ist nicht
zweifelsfrei bekannt (LiPo oder 18650 Li-Ion). `3450mV` ist der konservative
LiPo-sichere Wert — bei tatsächlichem Li-Ion-Akku löst das nur etwas früh
aus, während ein zu niedriger (Li-Ion-typischer, ~2800–3000mV) Cutoff einen
tatsächlichen LiPo-Akku tiefentladen und beschädigen könnte.

---

## Lift-Board (`MicroRosServoControlBoard`, Node `/ScheerenLift_Node`)

Aktuell nur Grundgerüst/Stub-Topics ohne echte Hardware-Anbindung (Aktor-Typ
für den Scherentisch noch nicht entschieden):

| Topic | Typ | Richtung |
|---|---|---|
| `/lift/cmd` | `std_msgs/Float32` | Sub (Stub) |
| `/camera/tilt` | `std_msgs/Float32` | Sub (Stub) |
| `/vacuum/cmd` | `std_msgs/Bool` | Sub (Stub) |
| `/lift/state` | `std_msgs/String` | Pub (Stub, sendet `"UNKNOWN"`) |
| `/lift/endstop` | `std_msgs/Bool` | Pub (Stub) |
| `/vacuum/pressure_ok` | `std_msgs/Bool` | Pub (Stub) |

Wird ergänzt, sobald die reale Lift-/Servo-/Endschalter-/Vakuum-Logik steht.

---

## Hardware-Identifikation / USB

Siehe `idf-micro-ros-setup.md`, Abschnitt "Hardware-Identifikation" — Chip-
Details, MAC-Adressen, udev-Regel für stabile `/dev/ttyUSB-robot` /
`/dev/ttyUSB-lift`-Namen, und der ModemManager-Interferenz-Fix
(`sudo systemctl disable --now ModemManager` — auf diesem Pi ohne
WLAN-Dongle/Mobilfunk-Modem gefahrlos möglich).
