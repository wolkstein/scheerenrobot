# Scheerenrobot – Bedienungsanleitung

Referenz für den laufenden Betrieb: welche ROS2-Topics es gibt, wie man sie
benutzt. Abgrenzung zu den anderen Doku-Dateien:

- `ROBOT_PLAN.md` – Architektur, Mission, Hardware-Planung, Phasen-Fortschritt
- `idf-micro-ros-setup.md` – ESP-IDF-Build-/Flash-Referenz für die Firmware-Entwicklung
- `ROS_INTERFACE.md` – kompakte Topic/Typ/Feld-Referenz für ROS2-Code (`ros2_ws`), ohne Prosa/Beispiele
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

Steuert den Scherentisch (Endlosdreh-Servo + 2 Endschalter), das
Kamera-Pitch-Servo und das Vakuumpumpen-Relais. Design-Prinzip: die
Haupt-Kommando-Topics sind bewusst minimal (nur "auf"/"zu" bzw. Zielwinkel),
die komplette Fahrlogik (Endschalter überwachen, rechtzeitig stoppen,
Timeout erkennen) läuft eigenständig auf dem Board — kein Polling/Timing von
ROS-Seite nötig.

### Übersicht

| Topic | Typ | Richtung | Beschreibung |
|---|---|---|---|
| `/lift/cmd` | `std_msgs/Bool` | Sub | `true`=auf, `false`=zu |
| `/camera/tilt` | `std_msgs/Float32` | Sub | Ziel-Pitch in Grad (0–90) |
| `/vacuum/cmd` | `std_msgs/Bool` | Sub | `true`=Pumpe an, `false`=aus |
| `/scissor/jog` | `std_msgs/Int32MultiArray` | Sub | Notfall/Einrichtung, ignoriert Endschalter (siehe unten) |
| `/servo_config` | `std_msgs/Int32MultiArray` | Sub | PWM-Kalibrierung setzen, NVS-persistiert |
| `/servo_config_state` | `std_msgs/Int32MultiArray` | Pub, 5Hz | Aktuell aktive PWM-Kalibrierung, gleiches Werte-Layout wie `/servo_config` (siehe unten) |
| `/telemetry` | `std_msgs/Int32MultiArray` | Pub, 5Hz | Zustand + Rohdaten (siehe unten) |

### Pin-Belegung

| GPIO | Funktion | Beschaltung |
|---|---|---|
| 4 | Endschalter Scherentisch oben (Position "auf") | interner Pullup, aktiv-low (Standardverdrahtung, per Hardwaretest bestätigt) |
| 13 | Endschalter Scherentisch unten (Position "zu") | interner Pullup, aktiv-low |
| 25 | Vakuumpumpe | Digitalausgang (Relais/Transistor) **oder** PWM-Servo, per Kconfig `VACUUM_ACTUATION_MODE` (Build-Zeit-Wahl, siehe unten) |
| 32 | Scherentisch-Servo (Endlosdreher) | PWM 1000–2000µs, 1500µs=Stopp |
| 33 | Kamera-Pitch-Servo (konventionell) | PWM, Endpunkte kalibrierbar (0°/90°) |

**Vakuumpumpen-Ansteuerung (GPIO 25):** Ursprünglich ein direkter
Digitalausgang zu einem Relais-Transistor (**nicht galvanisch getrennt**).
Am Prototyp verursachte das Abschalten der Relaisspule trotz
Freilaufdiode/anderer elektrischer Vorkehrungen einen induktiven
Spannungsimpuls, der den ESP32 in einen unklaren Zustand versetzt hat —
deshalb wurde als Fix ein Servo eingebaut, das die Pumpe zwischen zwei
kalibrierten PWM-Endpositionen umschaltet, genau wie die Scherentisch-/
Kamera-Servos. Beide Varianten bleiben als Kconfig-Auswahl erhalten
(`idf.py menuconfig` → "Scherentisch / Kamera / Vakuum Hardware" →
"Vakuumpumpen-Ansteuerungsart"), da eine künftige Revision voraussichtlich
auf ein Halbleiterrelais wechselt. Die Auswahl ist reine Build-Zeit-Config
(kein Neu-Flash-freies Umschalten) — `/vacuum/cmd` und `vacuum_state` in
`/telemetry` bleiben in beiden Fällen ein einfaches an/aus (Bool), nur die
Hardware-Ansteuerung darunter ändert sich.

Alle Werte über `idf.py menuconfig` → "Scherentisch / Kamera / Vakuum
Hardware" änderbar (Kconfig-Defaults, siehe `main/Kconfig.projbuild`).

### `/lift/cmd` — Scherentisch auf/zu

```bash
ros2 topic pub --once /lift/cmd std_msgs/msg/Bool "{data: true}"   # auf
ros2 topic pub --once /lift/cmd std_msgs/msg/Bool "{data: false}"  # zu
```

Das Board fährt den Servo autonom bis zum passenden Endschalter oder bis
`lift_timeout_ms` (Default 8000ms) abläuft. Ist der Zielendschalter bereits
erreicht, ist der Befehl ein No-Op. Bei einem `ERROR_TIMEOUT` reicht ein
erneutes `/lift/cmd`-Kommando, um es noch mal zu versuchen — z.B. nachdem
das Hindernis per `/scissor/jog` beseitigt wurde.

**Wichtig:** Welche PWM-Richtung (Puls über oder unter `lift_pwm_stop_us`)
physisch "auf" bzw. "zu" bedeutet, hängt von der Verkabelung/Einbaulage ab.
Falls sich der Tisch beim `auf`-Kommando in die falsche Richtung bewegt:
`lift_direction_up_is_increase` (Index 8) per `/servo_config` umschalten —
zur Laufzeit, kein Neu-Build/Flash nötig.

### `/camera/tilt` — Kamera-Pitch

`std_msgs/Float32`, Zielwinkel 0–90° relativ zum Roboterframe (0°=eingefahren/
waagerecht, 90°=senkrecht nach oben). Wird direkt (ohne Zwischenschritte)
über die kalibrierten Pulsweiten-Endpunkte (`camera_pwm_min/max_us`) auf PWM
gemappt.

```bash
ros2 topic pub --once /camera/tilt std_msgs/msg/Float32 "{data: 45.0}"
```

### `/scissor/jog` — Einrichtung & Notfall

`std_msgs/Int32MultiArray`, **genau 4 Werte**: `[target, direction, amount,
raw_pulse_us]`. Ignoriert bewusst die komplette Endschalter-Logik — gedacht
für die Erst-Einrichtung (PWM-Endpunkte per `/telemetry` ablesen) und für
den Notfall, wenn sich mechanisch etwas verklemmt hat.

| Index | Feld | Werte | Bedeutung |
|---|---|---|---|
| 0 | `target` | `0`=Scherentisch-Servo, `1`=Kamera-Servo | welcher Aktor |
| 1 | `direction` | `-1` / `0` / `+1` | Richtung (roh, siehe unten; nur relevant wenn `raw_pulse_us`=0) |
| 2 | `amount` | siehe unten | Impulsstärke / Haltedauer |
| 3 | `raw_pulse_us` | `0` = nicht gesetzt, sonst µs | nur für target=0, siehe unten |

- **target=0, `raw_pulse_us`>0 (manuelles Antasten, empfohlen zum Suchen des
  Mittelwerts):** hält den Servo für `amount` ms exakt auf `raw_pulse_us` —
  `direction` und der konfigurierte Jog-Offset werden dabei ignoriert. So
  lässt sich direkt und ohne Umweg über `/servo_config` z.B. `1480`, `1510`,
  `1520` usw. ausprobieren. Weiterhin durch die harte Sicherheitsgrenze
  `SERVO_PULSE_ABS_MIN/MAX_US` begrenzt.
- **target=0, `raw_pulse_us`=0, direction=±1 (Jog-Impuls):** `amount` =
  Impulsdauer in ms (Default-Cap 500ms, `SCISSOR_JOG_MAX_PULSE_MS`), Servo
  läuft mit `lift_pwm_jog_offset_us` Abstand vom Stopp-Wert und stoppt
  danach automatisch selbst. `direction` ist hier die **rohe** PWM-Richtung
  (unabhängig von `LIFT_DIRECTION_UP_IS_INCREASE`) — funktioniert also auch,
  bevor diese Zuordnung überhaupt bekannt ist.
- **target=0, `raw_pulse_us`=0, direction=0 (Halte-Modus):** hält den Servo
  für `amount` ms exakt auf der aktuell konfigurierten `lift_pwm_stop_us`.
- **target=0, amount=0:** sofortiger Stopp — Servo hält ab sofort die
  Neutral-/Stopp-Pulsweite (PWM bleibt aktiv, siehe "Warum das PWM-Signal
  nie abgeschaltet wird" unten), unabhängig von den anderen Feldern.
- **target=1 (Kamera):** `amount` = Grad-Schritt, `direction` = `-1`/`+1`,
  bewegt die aktuelle Pulsweite direkt um diesen Schritt weiter (auch über
  die kalibrierten 0°/90°-Endpunkte hinaus, bis zur harten
  Sicherheitsgrenze `SERVO_PULSE_ABS_MIN/MAX_US`). `raw_pulse_us` wird für
  target=1 nicht ausgewertet.

```bash
# Scherentisch: 300ms exakt auf 1480us halten (manuelles Antasten)
ros2 topic pub --once /scissor/jog std_msgs/msg/Int32MultiArray "{data: [0, 0, 300, 1480]}"
# Scherentisch: 200ms Impuls in Richtung "+", konfigurierter Jog-Offset
ros2 topic pub --once /scissor/jog std_msgs/msg/Int32MultiArray "{data: [0, 1, 200, 0]}"
# Scherentisch: Sofortiger Stopp
ros2 topic pub --once /scissor/jog std_msgs/msg/Int32MultiArray "{data: [0, 0, 0, 0]}"
# Kamera: 5° in Richtung "-"
ros2 topic pub --once /scissor/jog std_msgs/msg/Int32MultiArray "{data: [1, -1, 5, 0]}"
```

### `/servo_config` — PWM-Kalibrierung persistieren

`std_msgs/Int32MultiArray`, **genau 12 Werte in fester Reihenfolge**, sonst
wird die Nachricht verworfen (Fehlerlog, kein Crash). Wird sofort wirksam
**und** ins NVS persistiert (Namespace `servocfg`, übersteht Reboot).

| Index | Feld | Einheit | Kconfig-Default | Bedeutung |
|---|---|---|---|---|
| 0 | `camera_pwm_min_us` | µs | `1000` | Pulsweite bei 0° Kamera-Pitch |
| 1 | `camera_pwm_max_us` | µs | `2000` | Pulsweite bei 90° Kamera-Pitch |
| 2 | `lift_pwm_stop_us` | µs | `1500` | Neutral-/Stopp-Pulsweite Scherentisch-Servo |
| 3 | `lift_pwm_run_offset_us` | µs | `400` | Offset ab Stopp für normale Fahrt (auf/zu) |
| 4 | `lift_pwm_jog_offset_us` | µs | `50` | Offset ab Stopp für `/scissor/jog`-Impulse |
| 5 | `lift_pwm_reengage_offset_us` | µs | `200` | Offset ab Stopp für automatisches Nachführen in `OPEN`/`CLOSED` |
| 6 | `lift_timeout_ms` | ms | `8000` | Max. Fahrzeit bis `ERROR_TIMEOUT` |
| 7 | `lift_endstop_active_low` | `0`/`1` | `1` | `1`=aktiv-low (Standard), `0`=aktiv-high — Polarität der Endschalter, zur Laufzeit korrigierbar |
| 8 | `lift_direction_up_is_increase` | `0`/`1` | `1` | `1`=Puls über Stopp fährt "auf", `0`=Puls unter Stopp fährt "auf" — Fahrtrichtung, zur Laufzeit korrigierbar |
| 9 | `vacuum_pwm_min_us` | µs | `1000` | Pulsweite am "Aus"-Endpunkt der Pumpe (nur PWM-Ansteuerungsmodus) |
| 10 | `vacuum_pwm_max_us` | µs | `2000` | Pulsweite am "An"-Endpunkt der Pumpe (nur PWM-Ansteuerungsmodus) |
| 11 | `vacuum_pwm_invert` | `0`/`1` | `0` | `1`=An/Aus-Zuordnung vertauscht (An→`vacuum_pwm_min_us`) — korrigiert spiegelverkehrten Servo-Einbau, ohne die Endpunkte selbst anzufassen |

Index 9–11 sind im Digitalausgang-Modus (`VACUUM_ACTUATION_DIGITAL`,
Standard) wirkungslos, werden aber trotzdem persistiert — die Nachricht
braucht immer alle 12 Werte, unabhängig vom gewählten Ansteuerungsmodus.

**`-1` = "diesen Wert unverändert lassen":** Da kein Feld hier je negativ
sein kann (alles Pulsweiten/Offsets in µs, ein Timeout in ms oder ein
0/1-Flag), ist `-1` an jeder beliebigen Position eindeutig als "nicht
gesetzt" nutzbar. Das Board füllt fehlende Werte vor dem Speichern aus dem
aktuell aktiven Kalibrierungssatz auf (siehe `/servo_config_state`) —
damit lässt sich gezielt **ein einzelnes Feld** ändern, ohne die anderen elf
Werte erst korrekt abschreiben zu müssen und dabei versehentlich einen
bereits richtigen Wert mit einem veralteten zu überschreiben:

```bash
# Nur vacuum_pwm_invert setzen, alles andere unangetastet lassen
ros2 topic pub --once /servo_config std_msgs/msg/Int32MultiArray \
  "{data: [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1]}"
```

`lift_pwm_jog_offset_us` ist bewusst klein (Default `50` → 1450/1550µs)
gewählt, um bei der Erst-Inbetriebnahme vorsichtig antasten zu können. Bei
Bedarf hier live erhöhen, sobald das Bewegungsverhalten bekannt/sicher ist.

Ein geänderter `lift_pwm_stop_us` wird sofort auf den physischen
Scherentisch-Servo angewendet, **sofern er gerade im Ruhezustand ist**
(`IDLE`/`OPEN`/`CLOSED`, kein aktives Fahren/Jog) — sonst würde `/telemetry`
weiter den zuvor angewendeten Wert zeigen, bis irgendeine unabhängige
Bewegung den neuen Wert übernimmt. Während einer aktiven Fahrt/eines Jogs
wird der neue Wert nur gespeichert, aber erst bei der nächsten Ruhephase
angewendet. Ein geänderter `vacuum_pwm_min/max_us`/`vacuum_pwm_invert` wird
(im PWM-Modus) ebenfalls sofort auf die Pumpe angewendet, entsprechend
ihrem zuletzt kommandierten an/aus-Zustand — unabhängig vom Scherentisch
gibt es hier keinen "aktive Fahrt"-Zustand, der das verzögern würde.

```bash
ros2 topic pub --once /servo_config std_msgs/msg/Int32MultiArray \
  "{data: [1050, 1980, 1500, 400, 50, 200, 8000, 1, 1, 1000, 2000, 0]}"
```

**Bekannt gute Kalibrierung (Stand 2026-08-22, per `/servo_config_state`
ausgelesen und am Board bestätigt)** -- zum schnellen Wiederherstellen nach
Experimenten, z.B. nach NVS-Verlust oder auf einem Ersatzboard:

```bash
ros2 topic pub --once /servo_config std_msgs/msg/Int32MultiArray \
  "{data: [1989, 973, 1490, 200, 50, 200, 12000, 0, 0, 1223, 1800, 1]}"
```

### `/servo_config_state` — aktuelle Kalibrierung auslesen

`std_msgs/Int32MultiArray`, 5Hz, exakt dasselbe 12-Werte-Layout wie
`/servo_config` (siehe Tabelle oben) -- nur eben als Pub statt Sub. Gibt
den gerade aktiven, aus NVS geladenen (oder per `/servo_config` zuletzt
gesetzten) Kalibrierungssatz zurück. Praktisch, um z.B. nach einem Reboot
oder wenn die letzte `/servo_config`-Nachricht nicht mehr in der
Bash-History steht, den aktuellen Stand einmalig abzulesen -- die
Ausgabe lässt sich unverändert wieder als `/servo_config`-Payload
verwenden:

```bash
ros2 topic echo /servo_config_state --once
```

`lift_endstop_active_low` und `lift_direction_up_is_increase` erlauben es,
Endschalter-Polarität bzw. Fahrtrichtung zur Laufzeit zu korrigieren, falls
Verkabelung/Einbaulage mal nicht zur Software-Annahme passen — ohne
Neu-Build/Flash (die entsprechenden Kconfig-Werte sind nur noch die
Startwerte beim allerersten Boot).

Typischer Ablauf zur Erst-Kalibrierung: mit `/scissor/jog` (target=1) die
Kamera langsam an die mechanischen 0°/90°-Anschläge fahren, dabei
`camera_pwm_us` in `/telemetry` ablesen, die gefundenen Werte per
`/servo_config` speichern.

### `/telemetry` — Sammel-Status

`std_msgs/Int32MultiArray`, 6 Werte, 5Hz. Bündelt bewusst mehrere
zusammengehörige Diagnose-/Rohwerte des Boards in einem Topic (anders als
die Kommando-Topics oben, die bewusst *nicht* kombiniert sind — siehe
Begründung weiter unten).

| Index | Feld | Werte | Bedeutung |
|---|---|---|---|
| 0 | `lift_state` | Enum, siehe unten | Zustand der Scherentisch-State-Machine |
| 1 | `endstop_up` | `0`/`1` | Endschalter oben aktuell ausgelöst |
| 2 | `endstop_down` | `0`/`1` | Endschalter unten aktuell ausgelöst |
| 3 | `lift_pwm_us` | µs | Aktuell ausgegebene Pulsweite Scherentisch-Servo |
| 4 | `camera_pwm_us` | µs | Aktuell ausgegebene Pulsweite Kamera-Servo |
| 5 | `vacuum_state` | `0`/`1` | Vakuumpumpen-Relais an/aus |

`lift_state`-Enum (siehe `lift.h`):

| Wert | Name | Bedeutung |
|---|---|---|
| 0 | `IDLE` | zwischen den Endschaltern, keine Bewegung aktiv |
| 1 | `MOVING_UP` | fährt Richtung "auf" |
| 2 | `MOVING_DOWN` | fährt Richtung "zu" |
| 3 | `OPEN` | oberer Endschalter erreicht, Servo hält Neutralstellung |
| 4 | `CLOSED` | unterer Endschalter erreicht, Servo hält Neutralstellung |
| 5 | `ERROR_TIMEOUT` | Endschalter innerhalb `lift_timeout_ms` nicht erreicht — vermutlich Verklemmer |

```bash
ros2 topic echo /telemetry
```

### Warum das PWM-Signal nie abgeschaltet wird (Scherentisch-Servo)

**Hardware-Test-Ergebnis (wichtig):** Eine frühere Version dieser Firmware
hat das PWM-Signal im Ruhezustand komplett abgeschaltet (keine Pulse mehr),
um Kriechen/Zittern bei gehaltener Stopp-Pulsweite zu vermeiden. Der reale
Test am Board zeigte aber: der verbaute (billige) Endlosdreh-Servo stoppt
bei fehlendem Signal **nicht** — er hält vermutlich intern den letzten
gültigen PWM-Wert und/oder interpretiert das fehlende Signal als
undefinierten Zustand, in dem er unkontrolliert weiterlief (einmal bis zum
mechanischen Anschlag oben und wieder zurück nach unten, Stromkreis musste
unterbrochen werden). Deshalb gilt jetzt: **das Board hält immer einen
gültigen periodischen Puls auf der Leitung** — "Stopp" bedeutet die
Neutral-/Stopp-Pulsweite (`lift_pwm_stop_us`) zu senden, niemals Stille. Das
Kamera-Servo (Positions-Servo) war davon nie betroffen — der soll seinen
Winkel ohnehin aktiv halten.

**Nachführung in `OPEN`/`CLOSED`:** Die Gewindespindel des Scherentisches ist
selbsthemmend, verdreht sich also durch die Last auf dem Tisch nicht von
allein. Sollte der Endschalter trotzdem mal verlassen werden, erkennt das
Board das automatisch und fährt eigenständig zurück zum Endschalter — ganz
ohne ROS-Eingriff. Ein PID-Regler ist dafür nicht nötig, weil die
Endschalter mechanisch ein leichtes Überfahren (4-6mm) tolerieren. Diese
Nachführung nutzt einen eigenen, separat konfigurierbaren Offset
(`lift_pwm_reengage_offset_us`, Default `200`µs) statt des vollen
`lift_pwm_run_offset_us` — die Korrektur soll sanfter sein als eine bewusste
volle auf/zu-Fahrt.

**Vorsicht bei ersten Bewegungstests nach diesem Fix:** Die Telemetrie und
die Agent-Verbindung liefen beim beschriebenen Vorfall durchgehend stabil
(kein Disconnect) — die serielle Verbindung war also nicht die Ursache,
sondern tatsächlich das inzwischen entfernte PWM-Abschalten. Trotzdem
sicherheitshalber erstmal nur mit kleinem `lift_pwm_jog_offset_us` (Default
`50`, siehe `/servo_config`) und einzelnen kurzen `/scissor/jog`-Impulsen
testen, idealerweise mit vom mechanischen Antrieb entkoppeltem Servo, bevor
wieder am eingebauten Tisch getestet wird.

### Kalibrier-Workflow: Scherentisch-Mittelwert mit zwei Konsolen ermitteln

Ziel: die Pulsweite finden, bei der sich der Endlosdreh-Servo wirklich nicht
dreht (`lift_pwm_stop_us`, Startwert 1500µs). Dank `raw_pulse_us` in
`/scissor/jog` lässt sich direkt antasten, ohne vorher `/servo_config` zu
ändern.

**Konsole 1** — Telemetrie dauerhaft mitlaufen lassen:

```bash
ros2 topic echo /telemetry
```

**Konsole 2** — einzelne Testwerte antasten, jeweils kurz halten und danach
bewusst ein paar Sekunden abwarten/beobachten, bevor der nächste Wert
kommt (siehe Sicherheitshinweise oben):

```bash
ros2 topic pub --once /scissor/jog std_msgs/msg/Int32MultiArray "{data: [0, 0, 300, 1480]}"
# ... abwarten, /telemetry (lift_pwm_us) beobachten ...
ros2 topic pub --once /scissor/jog std_msgs/msg/Int32MultiArray "{data: [0, 0, 300, 1510]}"
# im Zweifel sofort stoppen:
ros2 topic pub --once /scissor/jog std_msgs/msg/Int32MultiArray "{data: [0, 0, 0, 0]}"
```

Sobald ein Wert gefunden ist, bei dem der Servo beim Halten wirklich still
steht, den gefundenen `lift_pwm_stop_us` (und bei Bedarf die anderen 5
Werte) per `/servo_config` dauerhaft speichern:

```bash
ros2 topic pub --once /servo_config std_msgs/msg/Int32MultiArray \
  "{data: [1000, 2000, 1495, 400, 50, 200, 8000, 1, 1, 1000, 2000, 0]}"
```
Bei kurzzeitig unklarem Verhalten sofort mit `{data: [0, 0, 0]}` stoppen.

### Warum die Kommando-Topics nicht zu einem Array kombiniert sind

`/lift/cmd`, `/camera/tilt` und `/vacuum/cmd` steuern drei unabhängige
Aktoren, die zu unterschiedlichen Zeitpunkten unabhängig voneinander
ausgelöst werden. Ein gemeinsames Array-Topic würde bedeuten, dass jede
Absicht (z.B. "nur Vakuum an") immer auch die aktuellen Werte der anderen
beiden mitschicken müsste, um sie nicht versehentlich zurückzusetzen — das
widerspricht dem Ziel, den Scherentisch so einfach wie möglich fernsteuern
zu können. Faustregel: ein Topic = ein unabhängiger Freiheitsgrad. Arrays
lohnen sich dort, wo die Werte wirklich eine zusammenhängende Einheit bilden
(z.B. `/scissor/jog`: Ziel+Richtung+Betrag als eine Anweisung, oder
`/telemetry`/`/servo_config`: mehrere zusammengehörige Diagnose- bzw.
Konfigurationswerte, die ohnehin immer gemeinsam gelesen/geschrieben werden).

---

## Hardware-Identifikation / USB

Siehe `idf-micro-ros-setup.md`, Abschnitt "Hardware-Identifikation" — Chip-
Details, MAC-Adressen, udev-Regel für stabile `/dev/ttyUSB-robot` /
`/dev/ttyUSB-lift`-Namen, und der ModemManager-Interferenz-Fix
(`sudo systemctl disable --now ModemManager` — auf diesem Pi ohne
WLAN-Dongle/Mobilfunk-Modem gefahrlos möglich).
