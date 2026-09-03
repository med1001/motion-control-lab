# Guide des paramètres

Ce document explique où configurer le projet **Motion Control Lab**, à quoi
correspond chaque paramètre et quelles unités utiliser. Les valeurs du scénario
de référence sont actuellement définies directement dans
`apps/motion_simulator.cpp` : il n'existe pas encore de fichier de configuration
externe.

## Vue d'ensemble

| Famille | Type ou emplacement | Rôle |
|---|---|---|
| Simulation | constantes dans `apps/motion_simulator.cpp` | Période, durée et résolution des encodeurs |
| Géométrie | `DifferentialDrive` | Rayon des roues et voie du robot |
| Moteur | `DcMotorParameters` | Modèle électrique et mécanique |
| Régulation | `PidConfig` | Asservissement de vitesse des roues |
| Navigation | `WaypointFollowerConfig` | Suivi des points de passage |
| Sécurité | `SafetyLimits` | Watchdog, courant et vitesse maximums |
| Trajectoire 1D | `TrapezoidalProfile` | Limites de vitesse et d'accélération |
| Scénario | `apps/motion_simulator.cpp` | Pose initiale, waypoints et perturbations |

## Où renseigner les paramètres

Pour modifier uniquement le scénario de référence, renseigner les valeurs dans
`apps/motion_simulator.cpp`, avant la boucle principale. Il est préférable de ne
pas modifier les valeurs par défaut des fichiers `include/motion/*.hpp`, car ces
valeurs s'appliquent à tous les futurs utilisateurs de la bibliothèque.

Exemple de configuration explicite :

```cpp
const motion::DcMotorParameters motor_parameters{
    .resistance = 0.8,
    .inductance = 0.02,
    .torque_constant = 0.12,
    .back_emf_constant = 0.12,
    .inertia = 0.003,
    .viscous_friction = 0.003,
};

const motion::PidConfig velocity_pid{
    .kp = 0.9,
    .ki = 3.5,
    .kd = 0.012,
    .output_min = -24.0,
    .output_max = 24.0,
    .derivative_filter_time = 0.015,
};

const motion::WaypointFollower follower({
    .position_gain = 1.2,
    .heading_gain = 3.0,
    .final_heading_gain = 2.0,
    .max_linear_speed = 0.8,
    .max_angular_speed = 2.0,
    .position_tolerance = 0.03,
    .heading_tolerance = 0.05,
});

motion::SafetySupervisor safety({
    .command_timeout = 0.20,
    .max_current = 45.0,
    .max_wheel_speed = 90.0,
});
```

Les initialiseurs nommés rendent l'unité et le rôle de chaque valeur plus
faciles à vérifier. Si une initialisation positionnelle est utilisée, les
valeurs doivent respecter exactement l'ordre des champs dans le fichier
d'en-tête correspondant.

## Paramètres de simulation

Ces constantes sont définies au début de `apps/motion_simulator.cpp`.

| Paramètre | Valeur de référence | Unité | Définition et règle |
|---|---:|---|---|
| `kDt` | `0.002` | s | Période de la boucle de commande. `0.002 s` correspond à `500 Hz`. Doit être strictement positive. |
| `kDuration` | `30.0` | s | Durée totale du scénario. |
| `kWheelRadius` | `0.08` | m | Rayon effectif d'une roue. Doit être strictement positif. |
| `kTrackWidth` | `0.42` | m | Distance latérale entre les centres des roues gauche et droite. Doit être strictement positive. |
| `kEncoderTicksPerRevolution` | `2048.0` | ticks/tr | Nombre de ticks pour un tour de roue. Doit être strictement positif. |

Le rayon doit correspondre au rayon **effectif** de roulement. Une erreur sur le
rayon produit directement une erreur d'échelle sur la distance estimée. Une
erreur sur la voie produit surtout une erreur d'orientation pendant les virages.

## Paramètres du moteur DC

Le type `DcMotorParameters` est déclaré dans `include/motion/dc_motor.hpp`. Les
valeurs utilisées par le simulateur sont créées dans
`apps/motion_simulator.cpp`.

| Champ | Défaut de la bibliothèque | Référence du simulateur | Unité SI | Définition et contrainte |
|---|---:|---:|---|---|
| `resistance` | `1.0` | `0.8` | ohm (Ω) | Résistance de l'induit. Strictement positive. |
| `inductance` | `0.5` | `0.02` | henry (H) | Inductance de l'induit. Strictement positive. |
| `torque_constant` | `0.1` | `0.12` | N·m/A | Constante de couple. Strictement positive. |
| `back_emf_constant` | `0.1` | `0.12` | V·s/rad | Constante de force contre-électromotrice. Strictement positive. |
| `inertia` | `0.01` | `0.003` | kg·m² | Inertie équivalente ramenée à l'arbre. Strictement positive. |
| `viscous_friction` | `0.01` | `0.003` | N·m·s/rad | Coefficient de frottement visqueux. Positif ou nul. |

Le modèle calcule trois états : courant en ampères, vitesse angulaire en rad/s
et position angulaire en radians. Les paramètres doivent être ramenés au même
arbre mécanique que celui utilisé par le rayon de roue. Si un réducteur est
présent, il faut donc tenir compte de son rapport dans les paramètres
équivalents ou ajouter explicitement un modèle de transmission.

## Paramètres du PID de vitesse

Le type `PidConfig` est déclaré dans `include/motion/pid_controller.hpp`. Le
simulateur transmet actuellement la même configuration aux roues gauche et
droite, mais `SimulatedDrivetrain` accepte deux configurations différentes.

| Champ | Référence | Unité indicative | Définition et contrainte |
|---|---:|---|---|
| `kp` | `0.9` | V/(rad/s) | Gain proportionnel. Positif ou nul. |
| `ki` | `3.5` | V/rad | Gain intégral. Positif ou nul. |
| `kd` | `0.012` | V·s²/rad | Gain dérivé appliqué à la mesure. Positif ou nul. |
| `output_min` | `-24.0` | V | Tension minimale autorisée. Doit être inférieure à `output_max`. |
| `output_max` | `24.0` | V | Tension maximale autorisée. Doit être supérieure à `output_min`. |
| `derivative_filter_time` | `0.015` | s | Constante de temps du filtre passe-bas de la dérivée. Positive ou nulle ; `0` désactive le filtrage. |

La dérivée est calculée sur la mesure, ce qui évite un pic dérivé lors d'un
changement brusque de consigne. L'intégrateur applique un anti-windup
conditionnel lorsque la sortie est saturée.

Pour régler les deux côtés séparément :

```cpp
const motion::PidConfig left_pid{/* paramètres gauche */};
const motion::PidConfig right_pid{/* paramètres droite */};
motion::SimulatedDrivetrain drivetrain(
    motor_parameters, left_pid, right_pid);
```

## Paramètres du suivi de waypoints

Le type `WaypointFollowerConfig` est déclaré dans
`include/motion/waypoint_follower.hpp`. Sans configuration explicite,
`WaypointFollower` utilise les valeurs suivantes.

| Champ | Défaut | Unité | Définition et contrainte |
|---|---:|---|---|
| `position_gain` | `1.2` | s⁻¹ | Transforme la distance au waypoint en vitesse linéaire. Strictement positif. |
| `heading_gain` | `3.0` | s⁻¹ | Corrige l'orientation vers le waypoint pendant la translation. Strictement positif. |
| `final_heading_gain` | `2.0` | s⁻¹ | Corrige l'orientation finale demandée. Strictement positif. |
| `max_linear_speed` | `0.8` | m/s | Limite de vitesse linéaire du châssis. Strictement positive. |
| `max_angular_speed` | `2.0` | rad/s | Limite de vitesse angulaire du châssis. Strictement positive. |
| `position_tolerance` | `0.03` | m | Distance maximale pour considérer la position atteinte. Strictement positive. |
| `heading_tolerance` | `0.05` | rad | Erreur maximale pour considérer l'orientation atteinte. Strictement positive. |

Une tolérance plus faible augmente la précision demandée, mais peut empêcher le
passage au waypoint suivant si le système oscille ou si la résolution des
encodeurs est insuffisante.

## Waypoints et pose initiale

Les waypoints sont définis dans `apps/motion_simulator.cpp` sous la forme :

```cpp
const std::vector<motion::Pose2d> waypoints{
    {1.0, 0.0, 0.0},
    {1.0, 1.0, std::numbers::pi / 2.0},
    {0.0, 1.0, std::numbers::pi},
    {0.0, 0.0, -std::numbers::pi / 2.0},
};
```

Chaque pose contient `{x, y, heading}` :

| Champ | Unité | Définition |
|---|---|---|
| `x` | m | Position selon l'axe horizontal global. |
| `y` | m | Position selon l'axe vertical global. |
| `heading` | rad | Orientation dans le plan. `0` pointe vers `+x`; les angles positifs tournent dans le sens trigonométrique. |

Conversions usuelles : `π/2 = 90°`, `π = 180°` et `-π/2 = -90°`.

La pose réelle et la pose estimée commencent actuellement à `{0, 0, 0}` :

```cpp
motion::Pose2d true_pose;
motion::Pose2d estimated_pose;
```

Pour utiliser une autre pose initiale, initialiser les deux poses et aligner
l'odométrie :

```cpp
motion::Pose2d true_pose{0.5, 0.5, std::numbers::pi / 2.0};
motion::Pose2d estimated_pose = true_pose;
odometry.reset(estimated_pose);
```

La liste des waypoints ne doit pas être vide, car la simulation accède toujours
à `waypoints[target_index]`.

## Paramètres de sécurité

Le type `SafetyLimits` est déclaré dans
`include/motion/safety_supervisor.hpp`. Le scénario de référence utilise
`{0.20, 45.0, 90.0}`.

| Champ | Défaut de la bibliothèque | Référence du simulateur | Unité | Définition et contrainte |
|---|---:|---:|---|---|
| `command_timeout` | `0.25` | `0.20` | s | Âge maximal de la dernière commande. Strictement positif. |
| `max_current` | `40.0` | `45.0` | A | Valeur absolue maximale du courant d'une roue. Strictement positive. |
| `max_wheel_speed` | `80.0` | `90.0` | rad/s | Valeur absolue maximale de la vitesse d'une roue. Strictement positive. |

Un dépassement ou un retour non numérique déclenche un défaut mémorisé. Le
drivetrain est alors désactivé et les deux PID sont réinitialisés. Un retour à
l'état normal nécessite un appel explicite à `SafetySupervisor::reset()`.

Le timeout doit rester nettement supérieur à `kDt` et prendre en compte les
retards réels du transport lorsqu'un adaptateur matériel remplace la simulation.

## Profil de mouvement trapézoïdal

`TrapezoidalProfile`, déclaré dans
`include/motion/trapezoidal_profile.hpp`, reçoit trois arguments :

```cpp
motion::TrapezoidalProfile profile(
    distance,
    max_velocity,
    max_acceleration);
```

| Argument | Unité | Définition et contrainte |
|---|---|---|
| `distance` | m ou rad | Déplacement signé demandé. Une valeur négative inverse le mouvement. |
| `max_velocity` | m/s ou rad/s | Vitesse maximale en valeur absolue. Strictement positive. |
| `max_acceleration` | m/s² ou rad/s² | Accélération maximale en valeur absolue. Strictement positive. |

Les unités dépendent de l'axe modélisé, mais elles doivent rester cohérentes
entre les trois arguments. Une distance courte génère automatiquement un profil
triangulaire ; une distance assez longue génère un profil trapézoïdal avec une
phase à vitesse constante.

Ce profil est actuellement disponible dans la bibliothèque et couvert par les
tests, mais il n'est pas connecté au scénario `motion_simulator`.

## Perturbation et injection de défaut

Le scénario applique une charge sur la roue droite entre 8 s et 10 s :

```cpp
const double right_load = time >= 8.0 && time < 10.0 ? 0.12 : 0.0;
```

`right_load` est un couple résistant en N·m. Les arguments de
`drivetrain.step()` sont, dans l'ordre, le temps, le pas, la charge gauche et la
charge droite :

```cpp
drivetrain.step(time, kDt, 0.0, right_load);
```

L'option de ligne de commande `--inject-timeout` interrompt le rafraîchissement
des commandes entre 18 s et 18,35 s. Ces bornes sont définies dans
`apps/motion_simulator.cpp` et peuvent être modifiées pour construire un autre
scénario de panne.

## Sortie et télémétrie

Par défaut, le simulateur écrit `out/telemetry.csv`. Un autre chemin peut être
passé comme premier argument :

```bash
./build/motion_simulator out/mon_essai.csv
./build/motion_simulator out/defaut.csv --inject-timeout
```

La boucle écrit une ligne toutes les dix itérations. Avec `kDt = 0.002 s`, la
fréquence d'export est donc de `50 Hz`. Modifier le diviseur `10U` dans la
condition d'export permet de changer cette fréquence sans modifier la fréquence
de commande.

## États et commandes d'exécution

Les types suivants représentent des données dynamiques, et non des paramètres
fixes à renseigner une seule fois :

- `MotorCommand` : numéro de séquence, consignes de vitesse gauche/droite et
  autorisation des moteurs ;
- `MotorFeedback` : horodatage, vitesses, courants, positions et tensions ;
- `MotorState` : courant, vitesse angulaire et position angulaire internes ;
- `Pose2d`, `Twist2d` et `WheelSpeeds` : pose, commande du châssis et vitesses
  des roues.

Dans un système réel, ces valeurs doivent être alimentées à chaque cycle par le
contrôleur, les encodeurs et le pilote matériel. Les champs `sequence` et
`timestamp` permettent de détecter des données anciennes ou perdues.

## Procédure recommandée après modification

Après chaque changement de paramètres :

1. reconstruire le projet ;
2. exécuter les tests ;
3. lancer une simulation nominale ;
4. vérifier la télémétrie, les saturations et les marges de sécurité ;
5. exécuter le scénario avec timeout.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/motion_simulator out/telemetry.csv
./build/motion_simulator out/fault_telemetry.csv --inject-timeout
python tools/plot_telemetry.py out/telemetry.csv --output docs/motion_control.png
```

Les valeurs du scénario de référence décrivent un modèle logiciel déterministe.
Elles ne doivent pas être utilisées sur un robot réel avant validation des
unités, identification du moteur, réglage du PID et vérification indépendante
des limites de sécurité.
