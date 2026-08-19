# Scheerenrobot – VTOL Retrieval System

## Mission

Der Roboter holt ein VTOL-Senkrechtstarter vom Landeplatz und bringt es zur Ladestation im Hanger.
Keine GNSS-Navigation – ausschließlich AprilTag-basierte Lokalisierung.

---

## Hardware

### Fahrgestell (vorhanden)
- Mecanum-Wheel-Chassis, 4 Motoren mit Encodern
- ESP32-S3 #1 (micro-ROS Serial, 921600 Baud, UART0 Pin 43/44)
  - ICM42670P IMU
  - 4× Motorsteuerung mit PID
  - Topics: `/cmd_vel`, `/odom_raw`, `/imu`, `/beep`, `/calibrate`
- RPi5 – ROS2 Jazzy (Docker)

### Kamera (vorhanden)
- OAK-D IoT 40 (RVC2, Stereo OV9282 + AF IMX378 Hauptkamera)
- Schwenkbar: 0° (vorne) ↔ 90° (oben), -z Achse
- Aktuell: Stereo-Bilder 640×400 @ 30fps via `oak_d_lite` Package

### Scherentisch + Kameraschwenk (ausstehend – Teile fehlen)
- ESP32 #2 (micro-ROS Serial, wie #1)
  - Scherentisch-Servomotor + Endschalter (oben/unten)
  - Kamera-Schwenkservo (0°–90°)
  - Relais-Ausgang → Vakuumpumpe (Saugnäpfe zur VTOL-Fixierung)

### VTOL (Zielobjekt)
- Geometrie bekannt → URDF
- AprilTags:
  - Seitenruder links + rechts (Groberkennung, Kamera vorne)
  - Rumpfunterseite Mitte, nach unten gerichtet (Feinausrichtung, Kamera oben)
- Motorgondeln = Forbidden Zones (bekannte Positionen relativ zum VTOL-Frame)

### Umgebung
- Hanger: AprilTags innen (Ladeposition) + außen (Einfahrt)
- Landeplatz: AprilTags an 4 Außenecken
- Flächenumrandung: virtuell konfigurierbar (YAML), flach

---

## ROS2 Packages (Zielarchitektur)

| Package | Status | Funktion |
|---|---|---|
| `scheerenbot_teleop` | ✓ vorhanden | Keyboard-Steuerung |
| `oak_d_lite` | ✓ vorhanden | Stereo-Kamera Node (depthai 2.x) |
| `scheerenbot_description` | ausstehend | URDF Roboter |
| `vtol_description` | ausstehend | URDF VTOL + Tag-Frames + Gondola-Frames |
| `scheerenbot_bringup` | ausstehend | Launch-Files für gesamtes System |
| `scheerenbot_localization` | ausstehend | EKF (Odom+IMU), AprilTag→Map |
| `scheerenbot_navigation` | ausstehend | Nav2 Konfiguration, Costmap, Waypoints |
| `scheerenbot_mission` | ausstehend | BehaviorTree.CPP State Machine |
| `scheerenbot_lift` | ausstehend | Scherentisch + Vakuum Interface (ESP32 #2) |

---

## TF2 Frame-Baum

```
map
├── odom
│   └── base_link
│       └── camera_link  (schwenkbar)
├── hangar_origin         (statisch, aus Hanger-Tags)
│   └── hangar_dock       (Ladekontakte-Position)
├── landing_pad_origin    (statisch, aus Ecken-Tags errechnet)
└── vtol_base             (dynamisch – wenn VTOL gefunden)
    ├── vtol_rudder_left  (aus VTOL-URDF)
    ├── vtol_rudder_right
    ├── vtol_belly_tag    (nach unten, Feinausrichtung)
    ├── vtol_gondola_left (→ Forbidden Zone Radius)
    ├── vtol_gondola_right
    └── vtol_pickup_point (Scherentisch-Ziel)
```

---

## ROS2 Topics / Actions (Ziel)

### Vorhanden (ESP32 #1)
| Topic | Typ | Richtung |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | → ESP32 |
| `/odom_raw` | `nav_msgs/Odometry` | ← ESP32 |
| `/imu` | `sensor_msgs/Imu` | ← ESP32 |
| `/calibrate` | `geometry_msgs/Vector3` | → ESP32 (NVS) |
| `/beep` | `std_msgs/UInt16` | → ESP32 |

### Geplant (ESP32 #2)
| Topic | Typ | Richtung |
|---|---|---|
| `/lift/cmd` | `std_msgs/Float32` (0.0–1.0) | → ESP32 |
| `/lift/state` | `std_msgs/String` (UP/DOWN/MOVING/ERROR) | ← ESP32 |
| `/lift/endstop` | `std_msgs/Bool` | ← ESP32 |
| `/camera/tilt` | `std_msgs/Float32` (0°–90°) | → ESP32 |
| `/vacuum/cmd` | `std_msgs/Bool` | → ESP32 |
| `/vacuum/pressure_ok` | `std_msgs/Bool` | ← ESP32 |

### Kamera
| Topic | Typ |
|---|---|
| `/camera/left/image_raw` | `sensor_msgs/Image` |
| `/camera/right/image_raw` | `sensor_msgs/Image` |
| `/camera/left/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth` | `sensor_msgs/Image` (geplant) |
| `/camera/points` | `sensor_msgs/PointCloud2` (geplant) |

---

## Mission State Machine

```
IDLE
  Roboter im Hanger, VTOL lädt
  └─ Trigger (manuell oder automatisch) ──►

LEAVE_HANGAR
  Hanger-Tags innen → Startpose in map
  Ausfahrt über bekannten Pfad
  Hanger-Tags außen → Re-Verifikation
  └─ Landeplatz sichtbar ──►

SEARCH_VTOL
  Kamera 0° (vorne), langsame Rotation
  AprilTag-Detektion: Seitenruder-Tags
  └─ Tags gefunden ──►

LOCALIZE_VTOL
  vtol_base in map publishen
  VTOL-URDF → alle Frames bekannt
  Gondola-Forbidden-Zones in Nav2-Costmap eintragen
  └─ Pose stabil ──►

PLAN_APPROACH
  Nav2 berechnet Pfad unter VTOL
  Pflichtweg um Gondeln herum (konfigurierbar per YAML)
  └─ Pfad OK ──►

NAVIGATE_UNDER_VTOL
  Nav2 fährt Pfad ab
  └─ vtol_pickup_point erreicht ──►

FINE_ALIGN
  Kamera 90° nach oben schwenken
  Bauch-AprilTag detektieren
  Mikro-Korrekturen bis Pose < Toleranz
  └─ Ausgerichtet ──►

LIFT_VTOL
  Vakuumpumpe an
  Druck OK? → Scherentisch hoch bis Endschalter oben
  └─ VTOL gesichert ──►

RETURN_TO_HANGAR
  Hanger-Tags außen → Lokalisierung
  Nav2 Einfahrt
  Hanger-Tags innen → Feinposition Ladekontakte
  └─ Dockposition erreicht ──►

DOCK
  Scherentisch runter bis Endschalter unten
  Vakuumpumpe aus
  └─ VTOL auf Ladekontakten ──►

IDLE
```

---

## Phasen-Plan

### Phase 1 – Lokalisierung (nächster Schritt)
- [ ] `stereo_image_proc` → Tiefenbild + Punktwolke
- [ ] AprilTag-Kalibrierung (CameraInfo mit echten Werten)
- [ ] `apriltag_ros` Package → Tag-Poses in TF2
- [ ] `robot_localization` EKF: Odom + IMU → `/odom`
- [ ] Statische Map (Hanger + Landeplatz) als YAML

### Phase 2 – VTOL-Erkennung
- [ ] VTOL URDF (Geometrie, Tag-Positionen, Gondola-Positionen)
- [ ] VTOL-Pose-Publisher: AprilTag → `vtol_base` in map

### Phase 3 – Navigation
- [ ] Nav2 Konfiguration (Costmap, Planner, Controller)
- [ ] Forbidden-Zone-Plugin für Gondeln
- [ ] Approach-Pfad als YAML-Waypoints

### Phase 4 – Scherentisch & Feinausrichtung (wenn ESP32 #2 vorhanden)
- [ ] ESP32 #2 Firmware (micro-ROS, Servo, Endschalter, Relais)
- [ ] `scheerenbot_lift` ROS2 Package
- [ ] Fine-Alignment-Node (Belly-Tag → Micro-Corrections)
- [ ] Kamera-Tilt-Controller

### Phase 5 – Mission Integration
- [ ] BehaviorTree.CPP State Machine
- [ ] Parameter-Server für Toleranzen, Pfade, Tag-IDs
- [ ] End-to-End-Test
- [x] udev-Regeln für stabile Device-Namen (Robot-Board `/dev/ttyUSB-robot`, Lift-Board `/dev/ttyUSB-lift` statt ttyUSB0/1) – siehe `idf-micro-ros-setup.md`
- [ ] Autostart nach Power-On: docker-compose (micro-ROS Agents + Kamera-Node), inkl. OAK-D-Kamera in die udev-Regeln aufnehmen

### Phase 6 – Visuelle Inspektion (n. Step)
- [ ] IMX378 Hauptkamera (AF, 12MP) für Detailaufnahmen
- [ ] Inspection-Node nach DOCK

---

## Build-Referenz

```bash
# ESP32 Firmware (kein ROS2 gesourced!)
. $IDF_PATH/export.sh
cd .../ybMecanumWheelMicroRosBot
idf.py build && idf.py flash

# micro-ROS Agent (Entwicklung: zwei einfache Container, feste udev-Namen)
# ttyUSB0/1 NICHT verwenden - beide CP2102-Chips melden dieselbe generische
# USB-Seriennummer, siehe idf-micro-ros-setup.md
# TODO fuer Autostart: docker-compose statt manueller docker run-Aufrufe
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  --name microros-agent-robot \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB-robot -b 921600

docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  --name microros-agent-lift \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB-lift -b 921600

# ROS2 Dev Container starten
docker start -ai ros2dev
# ODER neu:
docker run -it --net=host --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/wolke/develop/espidfmicroros/scheerenrobot/ros2_ws:/ros2_ws \
  --name ros2dev scheerenrobot-ros2dev bash

# Im Container
cd /ros2_ws && colcon build && source install/setup.bash

# Foxglove Bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# → ws://scheerenrobbi:8765
```
