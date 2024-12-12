Mission 2 : Détection d'obstacle 
===============================================
Cette partie correspond au code permettant de réaliser l'évitement d'obstacle.
----------------------------------------------------------------------------------------------
Dans cette mission, le TurtleBot doit naviguer sur un chemin délimité par des lignes blanche et jaune, tout en évitant des obstacles placés aléatoirement et en s’arrêtant devant une barrière fermant la route.

.. image:: consigne_mission2.png
   :alt: node_graph mission1
   :width: 600
   :align: center


Pour cette mission, nous avons réalisé notre propre code après avoir tenté d'utiliser, sans grand succès, les codes du tutoriel ROBOTIS que vous pouvez retrouver dans le github suivant : https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020/tree/main 

Nous avons donc exécuté la mission avec les codes du tutoriel ROBOTIS de la manière suivante : 

Étapes pour exécuter la mission
---------------------------------------------

Il faut commencer en démarrant ROS grâce à cette commande à éxécuter sur l'ordinateur : 

.. code-block:: bash

    roscore

Dans un second temps, ouvrez un terminal sur la RaspeberryPi (via SSH) du TurtleBot pour utiliser la commande suivante qui permet de lancer les configurations spécifiques pour le démarrage du robot TurtlBot3 : 

.. code-block:: bash

    roslaunch turtlebot3_bringup turtlebot3_robot.launch

Puis dans un second terminal sur le robot ouvert via SSH, lancez le noeud de publication des images de la caméra par la RaspberryPi embarquée dans le TurtleBot :

.. code-block:: bash

   roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

Ensuite, ouvrez un nouveau terminal, sur le navigateur cette fois-ci, pour lancer le noeud de calibration de la caméra intrinsèque puis celui de la calibration extrinsèque :

.. code-block:: bash

   roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

.. code-block:: bash

   roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

Dans un nouveau terminal sur l'ordinateur, utilisez la commande correspondant au noeud de la mission construction :

.. code-block:: bash

   roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=construction

Finalement, dans un dernier terminal, entrez la commande permettant d'éxécuter la mission construction :

.. code-block:: bash

   rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

Problèmes rencontrés:
---------------------------------------------
Malheureusement, cette mission, qui fonctionnait en simulation, n'a pas été un succès avec le vrai robot. En effet, le robot ne réussissait pas à détecter correctement les obstacles et se retrouvait à foncer dedans. 
Nous avons tenté de modifier un peu le code à plusieurs reprises, mais aucune tentative n'a fonctionné correctement.
