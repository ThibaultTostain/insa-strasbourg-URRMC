Installation de ROS1 Noetic
===========================

Instructions pour installer **ROS1 Noetic** sur Linux.

Ouvrir un terminal :
--------------------

Mettez à jour et mettez à niveau votre système avec les commandes suivantes :

.. code-block:: bash

    sudo apt-get update
    sudo apt-get upgrade

Téléchargez et exécutez le script d'installation de ROS Noetic :

.. code-block:: bash

    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
    chmod 755 ./install_ros_noetic.sh
    bash ./install_ros_noetic.sh

Installer les packages ROS :
----------------------------

Installez les packages supplémentaires requis pour les fonctionnalités ROS avec la commande suivante :

.. code-block:: bash

    sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

Installer les packages TurtleBot3 :
-----------------------------------

Ajoutez les packages spécifiques à TurtleBot3 :

.. code-block:: bash

    sudo apt install ros-noetic-dynamixel-sdk
    sudo apt install ros-noetic-turtlebot3-msgs
    sudo apt install ros-noetic-turtlebot3

Installer Gazebo :
------------------

Cloner le dépôt des simulations TurtleBot3 pour Gazebo, puis compiler le workspace :

.. code-block:: bash

    cd ~/catkin_ws/src/
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    cd ~/catkin_ws && catkin_make

Lancer une simulation test :
----------------------------

Pour tester la simulation, définissez le modèle TurtleBot3 et lancez un monde vide dans Gazebo :

.. code-block:: bash

    export TURTLEBOT3_MODEL=burger
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

Télecharger le package de la compétition :
------------------------------------------

Pour télécharger le package de la compétition, placez vous dans /catkin_ws clonez le dépôt suivant :

.. code-block:: bash

    cd ~/catkin_ws/src/
    git clone https://github.com/wafaesebbata/Turtlebot−simulation−for−Upper−Rhine−Robotic−Challenge.git
    cd ~/catkin_ws
    catkin_make
