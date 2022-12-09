# Application DRL au Bluerov_2 ROS/Unity

L'objectif est de pouvoir intégré un algorithme de Deep Reinforcement Learning au BlueRov2.
Le Bluerov2 est ici contrôlé via ROS Noetic. Unity est pour le moment utilisé seulement pour la visualisation. Le DRL est construit à partir des librairies gym et stable-baselines3.

Ce projet est en cours de construction, le code n'est pas propre.

## Prérequis

- Ubuntu 20.04.5 LTS
- Python 3.8
- Unity 2021.3.7f1

## Installations

### Installation de ROS Noetic

- Suivre les instructions de la doc officielle de ROS : http://wiki.ros.org/noetic/Installation/Ubuntu

### Installer le package DRL
1. Sourcer l'installation ROS :

        source /opt/ros/noetic/setup.bash


2. Cloner le répertoire :

        git clone https://github.com/Gregtmlg/Bluerov2_DRL.git

3. Initialiser le workspace :

        catkin init

4. Construire le workspace :

        catkin build

Si la commande échoue (peut arriver avec mavros) faire :

              catkin clean

Puis recommencer le build.



5. Permettre de sourcer automatiquement le workspace :

        echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
        echo 'source $HOME/Bluerov2_DRL/catkin_ws/devel/setup.bash' >> ~/.bashrc

6. Sourcer le workspace :

        source ~/.bashrc

### Installer PX4 :

- Suivre les instruction sur cette page jusqu'à "Configure Environment Variables" compris : https://hippocampusrobotics.github.io/fav_docs/installation/install_firmware.html

- **Attention:** les répertoires ne sont pas les même sur le site (fav = Bluerov2_DRL). Aussi ne pas réinstaller Mavros et Mavlink

- Au moment de "3. Build the code" dans la rubrique "Build The PX4 Firmware", si au bout de plusieurs lancements la commande ne compile plus, recommencer l'installation de PX4 et lancer la commande suivante à la place :
                
                DONT_RUN=1 make -j2 px4_sitl gazebo_uuv_bluerov2_heavy

### Cloner le projet Unity :

                git clone https://github.com/Gregtmlg/Bluerov2_Unity.git

- Lancer UnityHub, et ouvrir le projet Bluerov2_Unity avec le bon éditeur. 

### Ajouter ROS TCP Endpoint :

                cd ~/Blurov2_DRL/catkin_ws/src
                git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git


                cd cd ~/Blurov2_DRL/catkin_ws
                catkin build

### Ajouter l'extension ROS TCP Connector à Unity

- Suivre les instructions sur ce github : https://github.com/Unity-Technologies/ROS-TCP-Connector

### Installer les librairies python 

- le fichier requirements.txt liste toutes les librairies que j'avais sur ma machine au moment de faire fonctionner le Bluerov2.

## Guide d'utilisation

- Le programme se lance en éxecutant le script python stable_baselines_td3.py : 

                python3 ~/Bluerov2_DRL/TD3_VJET/stable_baselines_td3.py

- Dans un autre terminal, lancer le serveur tcp : 

                roslaunch ros_tcp_endpoint endpoint.launch

- Dans Unity lancer la simulation pour visualiser le Bluerov2.

- En cas de bug, ou pour arrêter le programme : 

                killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3


- Enfin pour mettre fin à la simulation, ou en cas de bug : 

        killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3 roscore