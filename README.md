# ProjetS6-STM32-SimPlane

Firmware STM32CubeIDE pour une maquette d'avion simplifiee basee sur une
`NUCLEO-L152RE` et une shield `X-NUCLEO-IKS01A3`. Le projet lit plusieurs
capteurs, pilote un afficheur 7 segments MAX7219, une barre de 8 LEDs et un
buzzer PWM, puis simule differents etats de vol et de moteur.

> Important: le code contient un mode de diagnostic afficheur pilote par
> `APP_DISPLAY_DIAG_ONLY` dans `Core/Src/main.c`. Avec la valeur `1`, la carte
> boucle sur des tests 7 segments. Passez cette constante a `0` pour activer la
> simulation complete de l'avion.

## Vue D'ensemble

Le firmware reproduit le comportement grossier d'un avion miniature:

- lecture de la temperature, de la pression, de l'humidite et du champ magnetique;
- lecture de deux potentiometres pour simuler la manette des gaz et la richesse;
- gestion d'un moteur virtuel avec etats `START`, `BROUT`, `STOP`;
- gestion du train d'atterrissage et des volets;
- affichage scrollant sur 7 segments;
- signalisation visuelle sur 8 LEDs;
- signal sonore via buzzer PWM;
- calibration inertielle au demarrage pour detecter une levation anormale de la carte.

## Materiel Requis

- `STM32 NUCLEO-L152RE`
- `X-NUCLEO-IKS01A3` pour les capteurs MEMS et environnementaux
- un afficheur 7 segments pilote par `MAX7219`
- une carte ou maquette type `ISEN32` avec:
  - 4 boutons (`BTN1` a `BTN4`)
  - 8 LEDs (`L0` a `L7`)
  - 2 potentiometres (`RV1` et `RV2`)
  - un buzzer

## Peripheriques Utilises

| Peripherique | Usage principal |
| --- | --- |
| `GPIO` | LEDs, boutons, chip select SPI |
| `EXTI` / `NVIC` | interruptions boutons |
| `ADC1` | lecture des deux potentiometres |
| `I2C1` | communication avec la shield IKS01A3 |
| `SPI1` | pilotage du MAX7219 |
| `TIM3` | generation PWM pour le buzzer |
| `TIM6` | base de temps pour les taches periodiques |
| `USART2` | traces de debug et `printf` |

## Cartographie Materielle

| Signal | Broche STM32 | Role |
| --- | --- | --- |
| `B1` | `PC13` | bouton bleu NUCLEO, changement de mode |
| `RV1` | `PA0` | manette des gaz |
| `RV2` | `PA1` | richesse |
| `BTN1` | `PA11` | bouton action / volets / moteur |
| `BTN2` | `PA12` | bouton action / train / carburant |
| `BTN3` | `PC6` | volets |
| `BTN4` | `PC5` | train d'atterrissage |
| `L0` a `L7` | `PB1`, `PB2`, `PB10`, `PB11`, `PB12`, `PB13`, `PB14`, `PB15` | barre de niveau |
| `SPI1` | `PA5` / `PA7` / `PA8` | `SCK` / `MOSI` / `CS` du MAX7219 |
| `I2C1` | `PB8` / `PB9` | liaison shield IKS01A3 |
| `TIM3 CH1` | `PB4` | buzzer PWM |

## Capteurs Supportes

Le firmware essaye d'initialiser plusieurs capteurs de la shield IKS01A3:

- `LSM6DSO` accelerometre et gyroscope;
- `LIS2DW12` accelerometre;
- `LIS2MDL` magnetometre;
- `HTS221` temperature et humidite;
- `LPS22HH` pression et temperature;
- `STTS751` temperature.

Au demarrage, l'application scanne aussi le bus `I2C1` et affiche le resultat
des detections dans la console serie.

## Modes Fonctionnels

Le bouton bleu `B1` de la NUCLEO change de mode. A chaque changement, la LED
correspondante s'allume pendant environ 1 seconde.

| Mode | Affichage 7 segments | LED associee | Effet principal |
| --- | --- | --- | --- |
| 1 | `TMP=xxC` | `L0` | temperature ambiante |
| 2 | `PRES=xxxxHPA` | `L1` | pression atmospherique |
| 3 | `MAG=xxxx` | `L2` | champ magnetique + jauge LED |
| 4 | `HUMI=xxPC` | `L3` | humidite + jauge LED |
| 5 | `RESERVE=xxPC` | `L4` | niveau de carburant + jauge LED |
| 6 | `HAUT` / `BAS` | `L5` | etat du train d'atterrissage |
| 7 | `UP` / `MID` / `DOWN` | `L6` | etat des volets |
| 8 | `ISEN` | `L7` | respiration de luminosite sur le MAX7219 |

Les textes longs defilent sur l'afficheur 7 segments.

## Commandes Utilisateur

### Bouton bleu `B1`

- appui court: passage au mode suivant;
- maintenu avec `BTN1`: demarrage du moteur si eteint, sinon baisse de carburant;
- maintenu avec `BTN2`: augmentation du carburant.

### Sans `B1`

- `BTN4`: train d'atterrissage en position basse;
- `BTN2`: train d'atterrissage en position haute;
- `BTN3`: volets vers le bas;
- `BTN1`: volets vers le haut.

Le comportement des volets suit la logique du code:

- `BTN3`: `UP -> MID`, sinon `DOWN`;
- `BTN1`: `DOWN -> MID`, sinon `UP`.

## Logique Moteur

Le moteur virtuel ne demarre que si:

- la richesse est a au moins `95%`;
- la manette des gaz est a au moins `20%`;
- le reservoir n'est pas vide.

Si les conditions ne sont pas remplies, le moteur passe en etat `BROUT` puis
s'arrete apres un delai. Lorsque le carburant devient faible, la simulation
ajoute des saccades aleatoires et fait clignoter la LED `L7` de plus en plus
vite. A `0%`, le moteur s'arrete avec un message `STOP` et un bip.

## Demarrage et Calibration

Au demarrage, le firmware:

1. initialise le HAL et l'horloge systeme;
2. configure les peripheriques GPIO, ADC, SPI, TIM3, TIM6 et USART2;
3. initialise l'afficheur MAX7219;
4. teste les sorties;
5. initialise et verifie les capteurs IKS01A3;
6. calibre la reference inertielle;
7. affiche `ISEN` par defaut.

Si la carte est levee alors que le moteur est a l'arret, une alerte `POSE`
apparait et le buzzer emet un signal pour signaler une situation non realiste.

## Build et Flash

### Prerequis

- `STM32CubeIDE` 6.16.x ou compatible;
- paquet de firmware `STM32Cube FW_L1 V1.10.6`;
- carte `NUCLEO-L152RE` reliee en `ST-LINK`.

### Etapes

1. Ouvrir le projet dans `STM32CubeIDE`.
2. Verifier que le workspace contient bien `Plane_Project`.
3. Compiler la configuration `Debug` ou `Release`.
4. Flasher la carte via `ST-LINK`.
5. Pour activer la simulation complete, mettre `APP_DISPLAY_DIAG_ONLY` a `0`
   dans `Core/Src/main.c` puis recompiler.

### Console de Debug

La sortie serie est configuree sur `USART2` en `115200 8N1`.

## Architecture du Code

Le code applicatif principal se trouve dans `Core/Src/main.c` et est organise
autour de taches periodiques:

- lecture ADC toutes les 50 ms;
- lecture des capteurs toutes les 200 ms;
- mise a jour de l'affichage environ toutes les 80 ms;
- gestion moteur / sorties / buzzer dans la boucle principale;
- interruption `TIM6` utilisee comme base temporelle.

Le driver de l'afficheur 7 segments se trouve dans `Drivers/display/`.

## Structure du Depot

```text
Core/               Code applicatif et startup STM32
Drivers/            HAL, CMSIS, drivers MEMS et MAX7219
MEMS/               Configuration X-CUBE-MEMS1
X-CUBE-MEMS1/       Fichiers fournis par ST pour les capteurs
ContexteSTM32/      Cahier des charges et documents de cours
```

Les dossiers `Debug/` et `Backups/` sont exclus du suivi Git pour garder le
depot propre et eviter de publier les artefacts de compilation.

## Documentation de Contexte

Le dossier `ContexteSTM32/` regroupe:

- le cahier des charges du projet;
- les supports de TP;
- les fiches techniques et documents ST utilises pendant le developpement.

## Licence

Le projet embarque des composants STMicroelectronics soumis a leurs licences
respectives. La licence du code applicatif du projet n'est pas encore definie.
Ajoutez une licence adaptee si vous souhaitez reutiliser ou redistribuer ce
travail.

## Remarques

- Le projet a ete genere avec STM32CubeIDE / CubeMX et un framework STM32Cube
  pour la famille L1.
- Si le bus I2C ne repond pas, le firmware tente une recuperation logique et
  affiche des informations de diagnostic dans la console serie.
- Si l'afficheur reste vide, verifier le cablage `SPI1` et le signal `CS`.
