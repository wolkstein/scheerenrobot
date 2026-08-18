# ESP-IDF / micro-ROS Setup – beide Controller-Boards

Referenz, um zwischen den zwei ESP32-Projekten zu wechseln, ohne die Settings
jedes Mal neu herzuleiten. ESP-IDF selbst wird in einer eigenen Konsole
gesourct (`. $IDF_PATH/export.sh`), **nicht** im ROS2-Docker-Container.

Da jedes Projekt sein eigenes `sdkconfig` und `build/`-Verzeichnis hat, merkt
sich jedes Projekt sein Target selbst – zum Wechseln reicht `cd` ins jeweilige
Verzeichnis, `idf.py set-target` ist nur beim allerersten Einrichten nötig.

---

## Übersicht

| | `ybMecanumWheelMicroRosBot` (Yahboom) | `MicroRosServoControlBoard` (Lift) |
|---|---|---|
| Chip | ESP32-S3 | ESP32 (D0WD-V3, klassisch) |
| Board | Yahboom Mecanum-Chassis-Board | BerryBase NodeMCU ESP32 Dev Board WROOM32 |
| IDF Target | `esp32s3` | `esp32` |
| Flash-Größe | 4MB | 4MB (per `esptool.py flash_id` verifiziert) |
| UART-Pins (TXD/RXD) | 43 / 44 | 1 / 3 |
| UART-Peripherie-Nummer | 2 | 2 |
| Baudrate | 921600 | 921600 |
| ROS Domain ID | 0 | 0 |
| USB-Port (üblich) | `/dev/ttyUSB0` | `/dev/ttyUSB1` |
| micro-ROS Komponente | `../extra_components/micro_ros_espidf_component` (geteilt) | `../extra_components/micro_ros_espidf_component_lift` (privat) |
| Komponenten-Branch | `jazzy` (v5.0.1) | `jazzy` (v5.0.1) |
| Node-Name | `YB_Car_Node` | `ScheerenLift_Node` |

---

## ybMecanumWheelMicroRosBot (ESP32-S3)

```bash
. $IDF_PATH/export.sh
cd ~/develop/espidfmicroros/scheerenrobot/ybMecanumWheelMicroRosBot
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Warum TXD/RXD = 43/44: Bei ESP32-S3-Devkits liegt die Boot-ROM-Konsole
(UART0) auf GPIO43/44, verbunden mit dem onboard USB-Bridge-Chip. Die
Firmware löscht den UART0-Treiber und installiert micro-ROS auf einer
zweiten UART-Peripherie-Instanz (`UART_NUM=2`), die per GPIO-Matrix auf
dieselben physischen Pins/denselben USB-Port gelegt wird.

Topics: `/cmd_vel`, `/odom_raw`, `/imu`, `/beep`, `/calibrate`.

---

## MicroRosServoControlBoard (ESP32 / BerryBase WROOM32)

```bash
. $IDF_PATH/export.sh
cd ~/develop/espidfmicroros/scheerenrobot/MicroRosServoControlBoard
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

Warum TXD/RXD = 1/3: Der klassische ESP32 hat nur GPIO0–39 – 43/44
existieren dort nicht. Die Boot-ROM-Konsole (UART0) liegt beim WROOM32 fest
auf GPIO1 (TX0) / GPIO3 (RX0), verbunden mit dem onboard USB-Serial-Chip.
Gleiches Muster wie beim S3-Board: UART0-Treiber löschen, micro-ROS auf
`UART_NUM=2` über dieselben physischen Pins/denselben USB-Port.

Topics (aktuell Stubs, siehe `main/main.c`): `/lift/cmd`, `/camera/tilt`,
`/vacuum/cmd` (Subscriber), `/lift/state`, `/lift/endstop`,
`/vacuum/pressure_ok` (Publisher).

**Wichtig – kein `printf` im Code:** Da micro-ROS denselben physischen
UART/USB-Port wie die Konsole nutzt, korrumpiert jede Klartext-Ausgabe
(`printf`) während der Transport läuft den binären XRCE-DDS-Stream und wirft
die Agent-Session raus. `ESP_LOGx` ist unkritisch (wird per
`esp_log_level_set("*", ESP_LOG_NONE)` gefiltert), `printf` **umgeht** diesen
Filter. Die `RCCHECK`/`RCSOFTCHECK`-Makros in diesem Projekt sind deshalb
bewusst still gehalten (kein `printf`).

**Weitere Falle – Transport-Auswahl der Komponente selbst:** Neben den
UART-Pins unseres Projekts (`main/Kconfig.projbuild`) hat die
micro-ROS-Komponente ein eigenes Kconfig-Menü mit einer Netzwerk-Interface-Wahl,
die standardmäßig auf **WLAN/UDP** steht statt auf UART. Unbedingt prüfen:
`idf.py menuconfig` → `Component config` → `micro-ROS Settings` →
`micro-ROS network interface select` → **"Micro XRCE-DDS over UART..."**
(`MICRO_ROS_ESP_UART_TRANSPORT`) auswählen, nicht "WLAN interface"
(`MICRO_ROS_ESP_NETIF_WLAN`, der Default). Genau diese Fehlkonfiguration
(Transport stand noch auf WLAN/UDP) war beim ersten Build-Versuch die
Ursache dafür, dass der Agent nicht handshaken konnte.

**Eigene, private micro-ROS-Komponente:** Anders als beim S3-Board wird hier
*nicht* die geteilte `extra_components/micro_ros_espidf_component` genutzt,
sondern eine eigene Kopie (`extra_components/micro_ros_espidf_component_lift`,
liegt auf Top-Level neben der anderen, ebenfalls Branch `jazzy`). Grund: Die
`libmicroros.a` ist architekturspezifisch kompiliert (Xtensa ESP32 vs.
ESP32-S3) und kann nicht zwischen beiden Boards geteilt werden. Falls
`colcon.meta` für dieses Board angepasst werden muss (z.B. mehr
Publisher/Subscriber), betrifft das nur diese private Kopie, nicht das
Yahboom-Board.

Falls die Quellen (`micro_ros_src`, `micro_ros_dev`) mal in einem kaputten
Zwischenzustand hängen (z.B. nach abgebrochenem Build – erkennbar an
`install/` ohne `lib/`-Unterordner): sauber neu bauen mit
```bash
cd ~/develop/espidfmicroros/scheerenrobot/extra_components/micro_ros_espidf_component_lift
make -f libmicroros.mk clean
```
und danach `idf.py build` erneut anstoßen (dauert dann wieder 10–20+ Min,
nicht unterbrechen).

---

## micro-ROS Agent (Entwicklung, RPi5)

Zwei Container, ein Board pro Port (siehe auch `startbefehl.txt`):

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  --name microros-agent-robot \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 921600

docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  --name microros-agent-lift \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB1 -b 921600
```

`/dev/ttyUSB0`/`/dev/ttyUSB1` sind nicht garantiert stabil über Reboots
hinweg (Enumerationsreihenfolge). TODO für Autostart: udev-Regeln für
stabile Device-Namen (siehe `ROBOT_PLAN.md`, Phase 5).
