# ESP-IDF / micro-ROS Setup – beide Controller-Boards

Referenz, um zwischen den zwei ESP32-Projekten zu wechseln, ohne die Settings
jedes Mal neu herzuleiten. ESP-IDF selbst wird in einer eigenen Konsole
gesourct (`. $IDF_PATH/export.sh`), **nicht** im ROS2-Docker-Container.

Da jedes Projekt sein eigenes `sdkconfig` und `build/`-Verzeichnis hat, merkt
sich jedes Projekt sein Target selbst – zum Wechseln reicht `cd` ins jeweilige
Verzeichnis, `idf.py set-target` ist nur beim allerersten Einrichten nötig.

**Falle – `idf.py set-target` auf ein bereits korrekt gesetztes Target NICHT
"vorsichtshalber" erneut ausführen:** Am 19.08. hat ein "sicherheitshalber"
erneut ausgeführtes `idf.py set-target esp32s3` (Target war schon esp32s3)
wegen einer IDF-Versions-Diskrepanz (`sdkconfig` war mit Init-Version 5.5.5
erzeugt, aktives Environment war 5.4.1) die komplette, projektspezifisch
abgestimmte `sdkconfig` (4MB Flash, 240MHz CPU, `FREERTOS_HZ=1000`, große
Partitionstabelle, ...) durch generische Defaults ersetzt. Da `sdkconfig` mit
im Projekt-Git eingecheckt ist, ließ sich das per `git checkout -- sdkconfig`
reparieren – trotzdem: `set-target` nur beim allerersten Einrichten eines
Projekts aufrufen, nicht routinemäßig beim Wechseln zwischen Projekten.

---

## Übersicht

| | `ybMecanumWheelMicroRosBot` (Yahboom) | `MicroRosServoControlBoard` (Lift) |
|---|---|---|
| Chip | ESP32-S3 | ESP32 (D0WD-V3, klassisch) |
| Board | Yahboom Mecanum-Chassis-Board | BerryBase NodeMCU ESP32 Dev Board WROOM32 |
| IDF Target | `esp32s3` | `esp32` |
| Flash-Größe | 4MB (per `esptool.py flash_id` verifiziert) | 4MB (per `esptool.py flash_id` verifiziert) |
| UART-Pins (TXD/RXD) | 43 / 44 | 1 / 3 |
| UART-Peripherie-Nummer | 2 | 2 |
| Baudrate | 921600 | 921600 |
| ROS Domain ID | 0 | 0 |
| USB-Port | `/dev/ttyUSB-robot` (udev, siehe unten) | `/dev/ttyUSB-lift` (udev, siehe unten) |
| micro-ROS Komponente | `../extra_components/micro_ros_espidf_component` (eigener Clone) | `../extra_components/micro_ros_espidf_component_lift` (eigener Clone) |
| Komponenten-Branch | `jazzy` (v5.0.1) | `jazzy` (v5.0.1) |
| Node-Name | `YB_Car_Node` | `ScheerenLift_Node` |

---

## Hardware-Identifikation (per `esptool.py --port <port> flash_id`)

| | `ybMecanumWheelMicroRosBot` (Yahboom) | `MicroRosServoControlBoard` (Lift) |
|---|---|---|
| Chip | ESP32-S3 (QFN56, Revision v0.2) | ESP32-D0WD-V3 (Revision v3.1) |
| Features | WiFi, BLE, Embedded PSRAM 2MB (AP_3v3) | WiFi, BT, Dual Core, 240MHz, VRef-Kalibrierung in eFuse, Coding Scheme None |
| Crystal | 40MHz | 26MHz (esptool meldet abweichend gemessene 15,44MHz – unkritische Warnung, kein reales Problem) |
| MAC | `74:4d:bd:93:aa:60` | `38:18:2b:e7:af:78` |
| Flash Manufacturer/Device | `5e` / `4016` | `68` / `4016` |
| Flash-Spannung | per eFuse auf 3.3V gesetzt | per Strapping-Pin auf 3.3V gesetzt |

Die MAC-Adresse ist der zuverlässigste Weg, ein geflashtes Board eindeutig
einer physischen Platine zuzuordnen (z.B. wenn beide gleichzeitig am RPi5
hängen und man sich nicht mehr sicher ist, welches Board gerade an welchem
Port steckt) – einfach `esptool.py --port /dev/ttyUSBx flash_id` aufrufen und
MAC gegen obige Tabelle prüfen.

**Warum `/dev/ttyUSBx` NICHT stabil ist:** Am 19.08. wurde beobachtet, dass
`/dev/ttyUSB1` das S3-Board (Yahboom) und `/dev/ttyUSB0` das klassische ESP32
(Lift) war – vorher war es genau umgekehrt dokumentiert. Ursache, per
`udevadm info -q property -n /dev/ttyUSB0` (bzw. `ttyUSB1`) verifiziert: beide
CP2102-USB-Bridge-Chips melden dieselbe generische Werks-Seriennummer
`0001` (`ID_SERIAL_SHORT=0001` bei **beiden**). Deshalb kann udev auch keinen
eindeutigen `/dev/serial/by-id`-Symlink für beide anlegen – nur eines der
beiden Boards taucht dort auf, das andere kollidiert namentlich. Die
`ttyUSBx`-Nummer hängt dadurch rein von der USB-Enumerationsreihenfolge beim
Booten/Einstecken ab und ist nicht vorhersagbar.

**Fix – udev-Regel gebunden an die physische USB-Buchse:** Auch ohne
eindeutige Chip-Seriennummer bleibt der physische USB-Pfad (welcher
USB-Controller/welche Buchse am Pi5) stabil, solange das Kabel stecken
bleibt. Per `udevadm info -q property -n /dev/ttyUSBx` (Feld `DEVPATH`) ermittelt:

| Board | Kernel-Pfad (`KERNELS=`) |
|---|---|
| Yahboom Robot-Board (S3) | `4-2` |
| Lift-Board (klassisch) | `2-2` |

Die beiden entsprechenden USB-Buchsen am Pi5 sind physisch markiert (Aufkleber/Beschriftung).
Regel unter `/etc/udev/rules.d/99-scheerenrobot-esp32.rules`:

```
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", KERNELS=="4-2", SYMLINK+="ttyUSB-robot"
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", KERNELS=="2-2", SYMLINK+="ttyUSB-lift"
```

Danach `sudo udevadm control --reload-rules && sudo udevadm trigger`. Ergebnis:
`/dev/ttyUSB-robot` und `/dev/ttyUSB-lift` sind feste Namen, unabhängig von
der Steckreihenfolge beim Booten – werden ab jetzt überall statt `ttyUSB0/1`
verwendet (Flash-Befehle unten, micro-ROS-Agent-Container,
`startbefehl.txt`).

Einschränkung: die Bindung hängt an der physischen Buchse, nicht am Chip
selbst – werden die Boards mal an andere Ports gesteckt (z.B. zum Debuggen),
muss die Regel angepasst oder zurückgesteckt werden. Robusterer, aber
aufwändigerer Alternativweg wäre, den CP2102-Chips per
Silicon-Labs-Konfigurationstool eine eindeutige Seriennummer ins EEPROM zu
schreiben – bisher nicht gemacht, da die Port-Bindung für den aktuellen
festverbauten Aufbau ausreicht.

---

## ybMecanumWheelMicroRosBot (ESP32-S3)

```bash
. $IDF_PATH/export.sh
cd ~/develop/espidfmicroros/scheerenrobot/ybMecanumWheelMicroRosBot
idf.py build
idf.py -p /dev/ttyUSB-robot flash monitor
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
idf.py -p /dev/ttyUSB-lift flash monitor
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

**Hinweis zur Kconfig-Netzwerk-Interface-Wahl:** Die micro-ROS-Komponente hat
ein eigenes Kconfig-Menü (`Component config` → `micro-ROS Settings` →
`micro-ROS network interface select`) mit einer Wahl zwischen WLAN/Ethernet/UART
(`MICRO_ROS_ESP_NETIF_WLAN` / `_ENET` / `MICRO_ROS_ESP_UART_TRANSPORT`). Für
unser `main.c` ist das **irrelevant** – der custom-serial-transport wird dort
unconditional per `rmw_uros_options_set_custom_transport()` registriert, ohne
dass eines dieser Kconfig-Symbole referenziert wird. Der eigentlich
entscheidende Schalter ist `RMW_UXRCE_TRANSPORT=custom` in `colcon.meta` (bzw.
`app-colcon.meta`, siehe unten) – der bestimmt, wie die vorgebaute
`libmicroros.a` compiliert wurde. Ein früherer Verdacht, die Kconfig-Wahl sei
Ursache eines gescheiterten Handshakes gewesen, hat sich als falsche Fährte
herausgestellt.

**Projektspezifische `colcon.meta`-Overrides ohne die Komponente anzufassen:**
Beide Boards folgen demselben Muster. `colcon.meta` (z.B. der
`rmw_microxrcedds`-Block mit `RMW_UXRCE_TRANSPORT`, `MAX_PUBLISHERS`,
`MAX_SUBSCRIPTIONS`, ...) liegt normalerweise direkt in der geclonten
Komponente – dort hineinzueditieren ist aber unpraktisch, weil ein frischer
`git clone` der Komponente diese Anpassungen wieder verwirft und man sie
schnell vergisst. Die Komponente bringt dafür bereits einen eingebauten
Override-Mechanismus mit (`CMakeLists.txt` der Komponente):

```cmake
set(APP_COLCON_META "${PROJECT_DIR}/app-colcon.meta")
```

`PROJECT_DIR` ist dabei das jeweilige ESP-IDF-Projektverzeichnis
(`ybMecanumWheelMicroRosBot/` bzw. `MicroRosServoControlBoard/`), nicht der
Komponentenordner. Existiert dort eine `app-colcon.meta`, wird sie beim Build
automatisch zusätzlich zur komponenteneigenen `colcon.meta` an colcon
übergeben (`--metas colcon.meta $(APP_COLCON_META)`) und überschreibt darin
enthaltene Pakete. Beide Projekte haben deshalb ihre eigene
`app-colcon.meta` mit nur dem `rmw_microxrcedds`-Block
(`custom`-Transport, 2 Nodes, 3 Publisher, 4 Subscriber, ...) – die
Komponenten selbst bleiben dadurch unveränderte, saubere Clones von `jazzy`,
Vorteil bei einem zukünftigen Re-Clone oder Update auf einen neueren
micro-ROS-Branch. `app-colcon.meta` gehört (anders als `extra_components/`)
mit ins jeweilige Projekt-Git.

Nach einer Änderung an `app-colcon.meta` merkt `idf.py build` eine reine
Inhaltsänderung u.U. nicht zuverlässig (gleiches Make-Dependency-Problem wie
bei `micro_ros_src`/`micro_ros_dev`, siehe unten) – sicherheitshalber einmal
clean bauen (Pfad zur jeweiligen Komponente anpassen):
```bash
cd ~/develop/espidfmicroros/scheerenrobot/extra_components/<komponente>
make -f libmicroros.mk clean
cd ~/develop/espidfmicroros/scheerenrobot/<projekt>
idf.py build
```

**Eigene, private micro-ROS-Komponente pro Board:** Jedes Board hat seine
eigene Komponenten-Kopie (`extra_components/micro_ros_espidf_component` für
Yahboom, `extra_components/micro_ros_espidf_component_lift` für Lift, beide
Top-Level, beide Branch `jazzy`) statt eine gemeinsam zu nutzen. Grund: Die
`libmicroros.a` ist architekturspezifisch kompiliert (Xtensa ESP32 vs.
ESP32-S3) und kann nicht zwischen beiden Boards geteilt werden. `colcon.meta`-
Anpassungen für ein Board betreffen so nur dessen eigene Kopie.

**Falle – `EXTRA_COMPONENT_DIRS` auf das ganze `extra_components/`-Verzeichnis
statt auf die konkrete Unterkomponente zeigen lassen:** ESP-IDF behandelt
einen `EXTRA_COMPONENT_DIRS`-Eintrag, der selbst kein Komponenten-Root ist
(kein direktes `CMakeLists.txt`), als Suchpfad und registriert **alle**
Unterordner mit eigenem `CMakeLists.txt` als Komponenten. Ursprünglich zeigte
`ybMecanumWheelMicroRosBot/CMakeLists.txt` auf `../extra_components` (das
ganze Verzeichnis) – solange dort nur eine Komponente lag, unproblematisch.
Seit `micro_ros_espidf_component_lift` als Geschwisterordner daneben liegt,
zog dieser Build **beide** Komponenten gleichzeitig rein, die intern
identisch benannte CMake-Targets definieren (`clean-microros`,
`libmicroros-prebuilt`, ...) → `CMake Error ... cannot create target
"clean-microros" because another target with the same name already exists`.
Fix: `EXTRA_COMPONENT_DIRS` in jedem Projekt exakt auf die eine benötigte
Unterkomponente zeigen lassen, nicht auf `extra_components` als Ganzes:
```cmake
set (EXTRA_COMPONENT_DIRS "../extra_components/micro_ros_espidf_component")       # Yahboom
set (EXTRA_COMPONENT_DIRS "../extra_components/micro_ros_espidf_component_lift")  # Lift
```

Falls die Quellen (`micro_ros_src`, `micro_ros_dev`) mal in einem kaputten
Zwischenzustand hängen (z.B. nach abgebrochenem Build – erkennbar an
`install/` ohne `lib/`-Unterordner): sauber neu bauen mit
```bash
cd ~/develop/espidfmicroros/scheerenrobot/extra_components/<komponente>
make -f libmicroros.mk clean
```
und danach `idf.py build` erneut anstoßen (dauert dann wieder 10–20+ Min,
nicht unterbrechen).

---

## micro-ROS Agent (Entwicklung, RPi5)

Zwei Container, ein Board pro festem udev-Device-Namen (siehe auch
`startbefehl.txt` und Abschnitt "Hardware-Identifikation" oben):

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  --name microros-agent-robot \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB-robot -b 921600

docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  --name microros-agent-lift \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB-lift -b 921600
```

`ttyUSB0`/`ttyUSB1` bewusst **nicht** verwendet – beide CP2102-Chips melden
dieselbe generische USB-Seriennummer, wodurch die Nummerierung von der
Enumerationsreihenfolge abhängt und instabil ist (siehe oben). Verbleibendes
TODO für Autostart: docker-compose statt manueller `docker run`-Aufrufe
(siehe `ROBOT_PLAN.md`, Phase 5).
