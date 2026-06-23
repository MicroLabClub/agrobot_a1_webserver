# agrobot_a1_webserver

# Agrobot A1 Web Dashboard

Agrobot A1 Web Dashboard este o aplicație web creată în Python Flask pentru monitorizarea și controlul unui robot/rover prin MAVLink. Proiectul afișează live date despre baterie, GPS, mișcare, siguranță, telecomandă RC, control WEB cu joystick și scanare LIDAR RoboPeak RPLIDAR A1.

Dashboard-ul permite urmărirea stării robotului în browser și controlul motoarelor prin comenzi MAVLink RC Override. LIDAR-ul este afișat ca radar live pentru detectarea obiectelor din jurul robotului.

## Funcții principale

* Monitorizare baterie 24V
* Grafic și tabel pentru tensiunea bateriei
* GPS live cu hartă Leaflet
* Viteză, heading, roll, pitch și yaw
* Citire canale RC
* Control WEB / telecomandă
* Joystick pentru motor spate și servo direcție
* Arm / Disarm / Manual mode prin MAVLink
* Radar LIDAR live pentru RoboPeak RPLIDAR A1M8-R5

## Porturi folosite

În configurația actuală:

```text
/dev/ttyACM0  = MAVLink / control motoare
/dev/ttyUSB0  = LIDAR RoboPeak RPLIDAR A1M8-R5
```

Aceste porturi pot fi schimbate din fișierul:

```text
config.py
```

## Instalare

Creează mediul virtual:

```bash
python3 -m venv venv
source venv/bin/activate
```

Instalează pachetele necesare:

```bash
pip install flask pymavlink rplidar pyserial
```

## Pornire

Conectează robotul și LIDAR-ul prin USB, apoi verifică porturile:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

Pornește serverul:

```bash
python3 app.py
```

Deschide în browser:

```text
http://10.9.0.10:5000
```

## Structura proiectului

```text
app.py          - serverul Flask și rutele API
config.py       - setări porturi, baterie, control și date robot
battery.py      - procesare date baterie
gps.py          - procesare date GPS
movement.py     - procesare mișcare și orientare
mission.py      - heartbeat, waypoint-uri și canale RC
control.py      - control motoare prin MAVLink RC Override
lidar.py        - citire și procesare date RPLIDAR
templates/      - interfața web HTML
```

## Notă de siguranță

Pentru testarea controlului motoarelor, robotul trebuie testat mai întâi cu roțile ridicate de la sol. Nu testa lângă oameni, animale sau obiecte fragile.
