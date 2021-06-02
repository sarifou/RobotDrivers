# Programme de control des robots mobiles T-quads

## Configurations
* Ubuntu mate 16.04.7 LTS
* ros kinetic
## Installations
### Installation de ubuntu mate
Télécharger ubuntu 3 et l'installer sur la carte mémoire de la raspberry pi.

Lien de téléchargement : [ici](https://releases.ubuntu-mate.org/archived/16.04/)
### Installation de ros kinectic
Etape 1 : Configurer de ubuntu pour qu'il accepte les packages ros

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
Etape 2 : Configurer les clés 

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
Etape 3 : Mettre à jour ubuntu

    sudo apt-get update
Etape 4 : Installer ROS, rqt, rviz et robot-generic librairies

    sudo apt-get install ros-kinetic-desktop

### Installations des packages additionnels ROS

* ROS Bridge
* ROS Serial
## Liste des topics

## Utilisation
Pour contrôler le tquad avec un client rosbridge

    roslaunch tquad tquad_bridge.launch

Pour contrôler le tquad avec le teleop_key

    roslaunch tquad tquad_teleop_key.launch
