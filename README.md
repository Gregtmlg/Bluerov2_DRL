# Application DRL au Bluerov_2 ROS/Unity

L'objectif est de pouvoir intégré un algorithme de Deep Reinforcement Learning au BlueRov2.
Le Bluerov2 est ici contrôlé via ROS Noetic, qui permet d'utiliser le package Mavros qui fait le lien avec PX4. Unity est pour le moment utilisé seulement pour la visualisation. Le DRL est construit à partir des librairies gym et stable-baselines3.

Ce projet est en cours de construction, le code n'est pas propre.

## Prérequis

- Ubuntu 20.04.5 LTS
- Python 3.8
- Unity 2021.3.13f1

## Structure

Le projet est divisé en 2 parties distinctes : une partie concernant ROS et le DRL, et une seconde concernant la visualisation avec Unity.

- ### ROS-DRL

La partie Ros-DRL nous sert au lancement de la simulation, à la gestion des messages ainsi qu'au contrôle du BlueRov2. Dans cette partie on retrouve notamment les fichiers de launch ros, les noeuds python gérant le contrôle du BlueRov2 (keyboard ou DRL). C'est aussi dans cette partie que l'on installe le logiciel de contrôle PX4.

ROS est un opérateur système robot qui nous permet de pouvoir développer notre application DRL de manière libre. Via MAVROS (et MAVLINK) notre application envoie les commandes que le drone doit exécuter et reçoit les données de "vol" via les topics ROS :

<img title="topics ros" alt="Alt text" src="./Bluerov2_DRL/images/ros_topics.png">

- ### Unity

Unity est un moteur de jeux vidéos très répandu. Il est très puissant et intéressant pour nous puisqu'il offre la possibilité de communiquer avec ROS en ajoutant un simple package au projet. 
Unity nous permet dans ce projet de créer/recréer l'environnement que l'on souhaite de manière la plus réaliste possible et d'y faire évoluer le Bluerov2 :

<img title="topics ros" alt="Alt text" src="./Bluerov2_DRL/images/Partie_Unity.png">


## Installations

### Installation de ROS Noetic

- Ajout de packages.ros.org à la liste des sources

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

- Installation du package complet ROS Noetic

        sudo apt update
        sudo apt install ros-noetic-desktop-full

- Automatisation des sources 

        source /opt/ros/noetic/setup.bash

        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

        source ~/.bashrc

- Dépendences python3/ROS

        sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

- Initialisation de rosdep

        sudo rosdep init

        rosdep update

- En cas de problème, suivre les instructions de la doc officielle de ROS : http://wiki.ros.org/noetic/Installation/Ubuntu


### Installation du package DRL-ROS

