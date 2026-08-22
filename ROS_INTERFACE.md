# Scheerenrobot – ROS2 Interface-Referenz

Kompakte, code-orientierte Referenz der ROS2-Schnittstelle beider Boards --
Topic, Typ, Richtung, Feld-Layout. Zum Schreiben/Ändern von ROS2-Code (Nodes,
Launch-Files, Tools im `ros2_ws`) gedacht, damit dafür nicht die komplette
Firmware beider Boards gelesen werden muss. Kein Ersatz für:

- `ROBOT_DOKU.md` – Bedienungsanleitung mit Beispielen, Sicherheitshinweisen,
  Kalibrier-Workflows und Begründungen (Quelle der Wahrheit bei Widersprüchen)
- `ROBOT_PLAN.md` – Architektur, Mission, Hardware-Planung
- `idf-micro-ros-setup.md` – ESP-IDF-Build-/Flash-Referenz

Beide Boards: `ROS_NAMESPACE=""`, `ROS_DOMAIN_ID=0` (Kconfig-Defaults, siehe
jeweiliges `Kconfig.projbuild`). Alle Subscriber mit fester Array-Länge
verwerfen Nachrichten falscher Länge kommentarlos (Fehlerlog auf dem Board,
kein Crash, keine Teilverarbeitung) -- IMMER die exakte Feldanzahl senden.

---

## Robot-Board (`ybMecanumWheelMicroRosBot`, Node `/YB_Car_Node`)

| Topic | Typ | Richtung | Rate | Beschreibung |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | – | Fahrbefehl: `linear.x`/`linear.y`/`angular.z` |
| `/odom_raw` | `nav_msgs/Odometry` | Pub | 11Hz | Odometrie aus Radencodern |
| `/imu` | `sensor_msgs/Imu` | Pub | 25Hz | ICM42670P Rohdaten |
| `/beep` | `std_msgs/UInt16` | Sub | – | `0`=aus, `1`=Dauerton, `10`-`10000`=Impuls in ms (auto-aus) |
| `/calibrate` | `geometry_msgs/Vector3` | Sub | – | `x`=Raddurchmesser mm, `y`=Spurbreite m, `z`=Radstand m. Sofort wirksam + NVS-persistiert (Namespace `calib`) |
| `/battery_state` | `sensor_msgs/BatteryState` | Pub | 1Hz | Live-Messwerte, siehe unten |
| `/battery_config` | `std_msgs/Int32MultiArray` | Sub | – | Pack-Konfiguration, 7 Werte, siehe unten. Sofort wirksam + NVS-persistiert (Namespace `battery`) |

### `/battery_state` Felder

| Feld | Bedeutung |
|---|---|
| `voltage` | Pack-Spannung in V |
| `percentage` | 0.0-1.0, linear zwischen Cutoff- und Max-Spannung |
| `power_supply_health` | `GOOD`(1) oder `DEAD`(3) |
| `power_supply_technology` | `LION`(2) oder `LIPO`(3) |
| `design_capacity` | Konfigurierte Kapazität in Ah |
| `temperature`, `current`, `charge`, `capacity` | immer `NaN` (keine Sensorik) |

### `/battery_config` Felder (genau 7 Werte)

| Index | Feld | Einheit | Default | Bedeutung |
|---|---|---|---|---|
| 0 | `cell_count` | Zellen | `2` | Zellen in Serie |
| 1 | `capacity_mah` | mAh | `2000` | Nennkapazität |
| 2 | `cell_voltage_max_mv` | mV/Zelle | `4200` | Vollgeladen-Spannung |
| 3 | `cell_voltage_warn_mv` | mV/Zelle | `3450` | Warnton-Schwelle |
| 4 | `cell_voltage_cutoff_mv` | mV/Zelle | `3450` | `DEAD`-Schwelle |
| 5 | `technology` | Enum | `3` | `2`=Li-Ion, `3`=LiPo |
| 6 | `adc_divider_factor_x1000` | Faktor×1000 | `4078` | Spannungsteiler-Kalibrierung |

Alle `cell_voltage_*_mv`-Werte sind **pro Zelle**, nicht Pack-weit.

---

## Lift-Board (`MicroRosServoControlBoard`, Node `/ScheerenLift_Node`)

| Topic | Typ | Richtung | Rate | Beschreibung |
|---|---|---|---|---|
| `/lift/cmd` | `std_msgs/Bool` | Sub | – | `true`=auf, `false`=zu. Autonome Fahrlogik auf dem Board (Endschalter/Timeout) |
| `/camera/tilt` | `std_msgs/Float32` | Sub | – | Ziel-Pitch 0-90°, linear auf `camera_pwm_min/max_us` gemappt |
| `/vacuum/cmd` | `std_msgs/Bool` | Sub | – | `true`=Pumpe an, `false`=aus (Digital- oder PWM-Ansteuerung, transparent) |
| `/scissor/jog` | `std_msgs/Int32MultiArray` | Sub | – | 4 Werte, Notfall/Einrichtung, ignoriert Endschalter, siehe unten |
| `/servo_config` | `std_msgs/Int32MultiArray` | Sub | – | 12 Werte, PWM-Kalibrierung setzen, siehe unten. Sofort wirksam + NVS-persistiert (Namespace `servocfg`). `-1` an einer Position = "unverändert lassen" |
| `/servo_config_state` | `std_msgs/Int32MultiArray` | Pub | 5Hz | Aktuell aktive Kalibrierung, exakt gleiches 12-Werte-Layout wie `/servo_config` (kein `-1`, immer konkrete Werte) |
| `/telemetry` | `std_msgs/Int32MultiArray` | Pub | 5Hz | 6 Werte, Status + Rohdaten, siehe unten |

### `/scissor/jog` Felder (genau 4 Werte): `[target, direction, amount, raw_pulse_us]`

| Index | Feld | Werte |
|---|---|---|
| 0 | `target` | `0`=Scherentisch-Servo, `1`=Kamera-Servo |
| 1 | `direction` | `-1`/`0`/`+1`, nur relevant wenn `raw_pulse_us`=0 |
| 2 | `amount` | target=0: Impuls-/Haltedauer ms (Cap `SCISSOR_JOG_MAX_PULSE_MS`, `0`=Sofortstopp); target=1: Grad-Schritt |
| 3 | `raw_pulse_us` | nur target=0: `>0` hält exakt diesen Puls für `amount` ms (überschreibt direction), `0`=nicht gesetzt |

Immer zusätzlich hart geclampt auf `[SERVO_PULSE_ABS_MIN_US, SERVO_PULSE_ABS_MAX_US]`.

### `/servo_config` und `/servo_config_state` Felder (genau 12 Werte, gleiche Reihenfolge)

| Index | Feld | Einheit | Default | Bedeutung |
|---|---|---|---|---|
| 0 | `camera_pwm_min_us` | µs | `1000` | Pulsweite bei 0° Kamera-Pitch |
| 1 | `camera_pwm_max_us` | µs | `2000` | Pulsweite bei 90° Kamera-Pitch |
| 2 | `lift_pwm_stop_us` | µs | `1500` | Neutral-/Stopp-Pulsweite Scherentisch-Servo |
| 3 | `lift_pwm_run_offset_us` | µs | `400` | Offset ab Stopp, normale Fahrt auf/zu |
| 4 | `lift_pwm_jog_offset_us` | µs | `50` | Offset ab Stopp, `/scissor/jog`-Impulse |
| 5 | `lift_pwm_reengage_offset_us` | µs | `200` | Offset ab Stopp, automatisches Nachführen in `OPEN`/`CLOSED` |
| 6 | `lift_timeout_ms` | ms | `8000` | Max. Fahrzeit bis `ERROR_TIMEOUT` |
| 7 | `lift_endstop_active_low` | `0`/`1` | `1` | `1`=aktiv-low |
| 8 | `lift_direction_up_is_increase` | `0`/`1` | `1` | `1`=Puls über Stopp fährt "auf" |
| 9 | `vacuum_pwm_min_us` | µs | `1000` | Pulsweite "Aus"-Endpunkt Pumpe (nur PWM-Modus) |
| 10 | `vacuum_pwm_max_us` | µs | `2000` | Pulsweite "An"-Endpunkt Pumpe (nur PWM-Modus) |
| 11 | `vacuum_pwm_invert` | `0`/`1` | `0` | `1`=An/Aus-Zuordnung vertauscht |

In `/servo_config` bedeutet `-1` an einem Index "diesen Wert unverändert
lassen" (kein Feld ist je legitim negativ) -- fehlende Werte werden vor dem
Speichern aus dem aktuell aktiven Stand ergänzt. `/servo_config_state` gibt
diesen aktiven Stand ohne `-1`-Platzhalter zurück und lässt sich unverändert
wieder als `/servo_config`-Payload verwenden.

### `/telemetry` Felder (6 Werte)

| Index | Feld | Werte | Bedeutung |
|---|---|---|---|
| 0 | `lift_state` | Enum, siehe unten | Zustand der Scherentisch-State-Machine |
| 1 | `endstop_up` | `0`/`1` | Endschalter oben aktuell ausgelöst |
| 2 | `endstop_down` | `0`/`1` | Endschalter unten aktuell ausgelöst |
| 3 | `lift_pwm_us` | µs | Aktuell ausgegebene Pulsweite Scherentisch-Servo |
| 4 | `camera_pwm_us` | µs | Aktuell ausgegebene Pulsweite Kamera-Servo |
| 5 | `vacuum_state` | `0`/`1` | Pumpe an/aus |

`lift_state`-Enum: `0`=`IDLE`, `1`=`MOVING_UP`, `2`=`MOVING_DOWN`, `3`=`OPEN`,
`4`=`CLOSED`, `5`=`ERROR_TIMEOUT`.
