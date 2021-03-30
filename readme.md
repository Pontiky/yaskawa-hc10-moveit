Projet de Master 2 Robotique Décision et Commande - Intégration d’un robot coopératif dans un environnement de production à l’aide du middleware ROS

=============

## A propos
Projet réalisé pour l'UE Projet en M2 RODECO (Paul Sabatier, Toulouse) en 2020-2021 par le groupe étudiant suivant : Gaëlic BECHU, Lucas CERE, Gérémy PEDOUSSAT, Matthieu PIERSON, Anthony THURIES
pour les clients de l'AIP PRIMECA Occitanie : Michel TAIX, Fabien MARCO
sur le robot collaboratif Yaskawa HC10.

## Requis

* Ubuntu 18 (recommandé)
* [ROS Melodic](http://wiki.ros.org/melodic)
* [MoveIt 1](https://moveit.ros.org/install/)

## Compilation

* Placez-vous à la racine du répertoire
* Ouvrez un terminal
* Tapez `source /opt/ros/melodic/setup.bash`
* Tapez `source devel/setup.bash`
* Compilez avec la commande `catkin_make`

## Lancement

* Visualiser le robot sous RViz : `roslaunch motoman_hc10_support test_hc10.launch`
* Lancer la simulation locale avec MoveIt : `roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch`
* Lancer la simulation connectée au robot avec MoveIt : `roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.0.113 controller:=yrc1000` (changer IP si besoin)

## Planification de mouvement

* Lancer une simulation avec MoveIt
* Déplacez le robot graphiquement avec le contrôle de l'organe terminal
* Allez dans l'onglet `Planning`, appuyez sur `Plan` pour vérifier le plan solution puis sur `Execute`

## Scripts Python

* Commande du robot en joints : `rosrun motoman_hc10_moveit_config move_to_joints.py '[1.57, 0, 0, 0, 0, 0]'` (joints en rad)
* Redéfinition des collisions avec la scène dans `scripts/add_interferences.py`
* Tâche en vitesse (non fonctionnel) dans `scripts/velocity_control.py`