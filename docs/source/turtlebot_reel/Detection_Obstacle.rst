Mission : Détection d'obstacle 
===============================================
Cette partie correspond au code permettant de réaliser l'évitement d'obstacle.
----------------------------------------------------------------------------------------------
Dans cette mission, le TurtleBot doit naviguer sur un chemin délimité par des lignes blanche et jaune, tout en évitant des obstacles placés aléatoirement et en s’arrêtant devant une barrière fermant la route.

.. image:: consigne_mission2.png
   :alt: node_graph mission1
   :width: 600
   :align: center


Pour cette mission, nous avons utiliser les codes du tutoriel ROBOTIS que vous pouvez retrouver dans le github suivant : https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020/tree/main

1 . Code de détection du damier 
-----------------------------



Explication rapide du code : 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


2 . Code du suivi de ligne  
-----------------------------



3 . Codes de calibration des caméras intrinsèques et extrinsèques 
---------------------------------------------------------------------



4 . Code "master_node"
-----------------------------




.. code-block:: bash
  

Explication rapide du code : 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 



5 . Fichier .launch
-----------------------------

Le fichier .launch en ROS est un fichier qui sert à démarrer et configurer plusieurs noeuds et paramètres de ROS en une seule commande.

Ici, ce fichier démarre le noeud spécifique 'core_node_mission' du package turtlebot3_autorace_core. 

.. code-block:: bash
    
    <launch>
         <node pkg="turtlebot3_autorace_core" type="core_node_mission" name="core_node_mission"                 output="screen" />
    </launch>

6. Etapes pour exécuter la mission
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
