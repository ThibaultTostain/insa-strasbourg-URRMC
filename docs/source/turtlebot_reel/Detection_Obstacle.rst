Mission : Détection d'obstacle 
===============================================
Cette partie correspond au code permettant de réaliser l'évitement d'obstacle.
----------------------------------------------------------------------------------------------
Dans cette mission, le TurtleBot doit naviguer sur un chemin délimité par des lignes blanche et jaune, tout en évitant des obstacles placés aléatoirement et en s’arrêtant devant une barrière fermant la route.

.. image:: consigne_mission2.png
   :alt: node_graph mission1
   :width: 600
   :align: center


Pour cette mission, nous avons utilisé les codes du tutoriel ROBOTIS que vous pouvez retrouver dans le github suivant : https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020/tree/main

Nous avons donc exécuté la mission avec les codes du tutoriel ROBOTIS de la manière suivante : 

Etapes pour exécuter la mission
---------------------------------------------

Dans un premier temps, ouvrez un terminal pour utiliser la commande suivante qui permet de lancer les configurations spécifiques pour le démarrage du robot TurtlBot3 : 

.. code-block:: bash

    roslaunch turtlebot3_bringup turtlebot3_robot.launch

Ensuite, ouvrez un nouveau terminal pour lancer le noeud de calibration de la caméra intrinsèque:

.. code-block:: bash

   roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

Dans un nouveau terminal, utilisez la commande correspondant au noeud de la mission construction :

.. code-block:: bash

   roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=construction

Finalement, dans un dernier terminal, entrez la commande permettant d'éxécuter la mission construction :

.. code-block:: bash

   rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

Problèmes rencontrés:
---------------------------------------------
Malheureusement, cette mission, qui fonctionnait en simulation, n'a pas été un succès avec le vrai robot. En effet, le robot ne réussissait pas à détecter correctement les obstacles et se retrouvait à foncer dedans. 
Nous avons tenté de modifier un peu le code à plusieurs reprises, mais aucune tentative n'a fonctionné correctement.
