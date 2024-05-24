# Documentatie voor AX-12 Servo Controller

## Inleiding

Deze documentatie biedt informatie over het aansturen van AX-12 servo's met behulp van de `servo_controller` node in ROS2.

## Vereisten

- Python 3
- ROS2 iron
- Het `RPi.GPIO` pakket (voor Raspberry Pi GPIO-aansturing)

## Installatie


1. Installeer de benodigde afhankelijkheden

   ```bash
   pip install RPi.GPIO
   ```

2. Zorg ervoor dat ROS2 geïnstalleerd en geconfigureerd is op uw systeem.

## Gebruik

1. Verbind uw AX-12 servo('s) met uw Raspberry Pi.

2. Start de ROS2 master node(nu niet nodig nog maar toekomst mischien wel)

   ```bash
   ros2 run roscore
   ```

3. Start de servo controller node

   ```bash
   ros2 run my_ax12_servo_controller my_ax12_node
   ```

4. Publiceer servo commando's naar het `servo_command` onderwerp in het volgende formaat vanaf een andere ros instalatie

   ```
   ros2 topic pub  --once /servo_command std_msgs/msg/String "{data: 'servo_id command duration position'}"
   
   ```

   - `servo_id` ID van de servo die je wilt aansturen.
   - `command` Aansturingscommando. zie https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-data-address terwijl `0` de servo stopt.
   - `duration` Duur (in milliseconden) gedurende welke de beweging moet plaatsvinden. Dit is alleen van toepassing als het commando 32(beweeg wheelmode) is.
   - `waarde` Gewenste waarde voor de servo voor het commando(voor 30 is het de hoek tussen 0-1023 voor commando 32 is het de snelheid en richting(0-1023-2048).

   Voorbeeld commando
   
   ```
   ros2 topic pub  --once /servo_command std_msgs/msg/String "{data: '1 30 1000 512'}"
   ```
   
   Dit commando beweegt servo met ID 1 naar positie 512.

## ROS2 Onderwerpen

### Gepubliceerde Onderwerpen

- `servo_controller_nodelog` (`std_msgsString`)

   Logberichten van de servo controller node.

### Geabonneerde Onderwerpen

- `servo_command` (`std_msgsString`)

   Commando's om de servo's aan te sturen.

## Servo Commando Bericht

### Berichtformaat

Het berichtformaat bestaat uit vier waarden gescheiden door spaties

```
servo_id command duration position
```

- `servo_id` Het ID van de servo die u wilt aansturen. Dit moet een geheel getal zijn.
- `command` Het aansturingscommando dat naar de servo moet worden gestuurd. Dit moet een geheel getal zijn. Niet-nul waarden geven aan dat de servo moet bewegen, terwijl `0` de servo stopt.
- `duration` De duur (in milliseconden) gedurende welke de beweging moet plaatsvinden. Deze parameter is optioneel en alleen van toepassing als het commando niet-nul is. Dit moet een geheel getal zijn.
- `position` De gewenste positie voor de servo. Dit moet een geheel getal zijn dat de positie binnen het bereik van de servo weergeeft.

### Voorbeeld

Hier is een voorbeeld van hoe u een servo commando kunt publiceren met behulp van het `ros2 topic pub` commando

```bash
ros2 topic pub --once servo_command std_msgsmsgString {data '5 32 1000 1300'}
```

Dit commando verplaatst de servo met ID `5` naar positie `1300` over een periode van `1000` milliseconden. Het commando is `32`, wat continu roteren aangeeft.

### Opmerkingen

- Zorg ervoor dat het servo ID, commando, duur en positie geldig zijn en binnen het acceptabele bereik liggen voor uw servo-opstelling.
- Pas de parameters aan volgens uw specifieke vereisten en servo-configuratie.

## Licentie

Dit project is gelicentieerd onder de [MIT-licentie](LICENSE).