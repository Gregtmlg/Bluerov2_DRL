# Application DRL au Bluerov_2 ROS/Unity

L'objectif est de pouvoir intégré un algorithme de Deep Reinforcement Learning au BlueRov2.
Le Bluerov2 est ici contrôlé via ROS Noetic. Unity est pour le moment utilisé seulement pour la visualisation. Le DRL est construit à partir des librairies gym et stable-baselines3.

## Prérequis

- Ubuntu 20.04.5 LTS
- Python 3.8
- Unity 2021.3.7f1

## Installation de ROS Noetic

Suivre les instructions de la doc officielle de ROS : http://wiki.ros.org/noetic/Installation/Ubuntu

## Installer le package DRL
1. Sourcer l'installation ROS :

        source /opt/ros/noetic/setup.bash


2. Créer le workspace :

        mkdir -p ~/Bluerov2_DRL/catkin_ws/src && cd ~/Bluerov2_DRL/catkin_ws

3. Initialiser le workspace :

        catkin init

4. Construire le workspace vide

        catkin build

5. Permettre de sourcer automatiquement le workspace :

        echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
        echo 'source $HOME/Bluerov2_DRL/catkin_ws/devel/setup.bash' >> ~/.bashrc

6. Sourcer les workspace :

        source ~/.bashrc

7. Cloner le répertoire :

        cd ~/Bluerov2_DRL/catkin_ws
        git clone https://github.com/FormulasAndVehicles/bluerov_sim.git src/bluerov_sim