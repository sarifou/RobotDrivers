# Programme de control des robots mobiles T-quads

## Configurations
* Raspbian
* ros kinetic
## Installations
### Installation de Raspbian
Télécharger Raspberry Pi OS Desktop et l'installer sur la carte mémoire de la raspberry pi.

Lien de téléchargement : [ici](https://www.raspberrypi.org/software/operating-systems/#raspberry-pi-desktop)
### Installation de ROS kinetic
Suivre le tutoriel suivant pour installer ROS depuis les sources.

Tutoriel d'installation : [ici](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)

### Installations des packages additionnels ROS

**ROS Bridge**

    sudo apt-get install ros-kinetic-rosbridge-server

**ROS Serial**

    sudo apt-get install ros-kinetic-rosserial

### Installation du paquet tquad ros
Créer et initialiser un espace de travail ros sur votre bureau

    mkdir ros_workspace/src
    cd ros_workspace/src
    catkin_make

Cloner le répertoire github

    git clone https://github.com/sarifou/RobotDrivers.git

Copier coller le répertoire tquad dans vers le src de ros_workspace et construiser à nouveau votre espace de travaille.

    catkin_make

### Installation du firmeware pour l'arduino méga.

Avec le logiciel arduino, compiler le fichier firmeware.ino dans l'arduino mega du tquad

## Liste des topics

## Utilisation
Pour contrôler le tquad avec un client rosbridge

    roslaunch tquad tquad_bridge.launch

Pour contrôler le tquad avec le teleop_key

    roslaunch tquad tquad_teleop_key.launch
