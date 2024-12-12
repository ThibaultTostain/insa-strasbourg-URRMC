Mission 4 : Parking
===============================================

Durant cette mission, le TurtleBot doit réussir une maneuvre de parking dans la place non occupée parmi les deux disponibles, puis sortir de la place après un délai.

.. image:: parking.png
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

Finalement, dans un dernier terminal, entrez la commande permettant d'éxécuter la mission construction :

.. code-block:: bash

   rosrun competition mission_parking.py

Piste d'amélioration :
---------------------------------------------

Ce code a été réalisé le deuxième jour de compétition, quelques minutes avant le passage. Il nécessite ainsi des améliorations, auxquelle nous avons pris la peine de réfléchir.

- Parfois, certaines mesures du Lidar sont défaillantes, ce qui ne permet pas de mesurer la distance souhaitée étant donné que le noeud qui envoi les données n'intègre pas de pré filtrage dans celui-ci. Ainsi, on peut envisager lors de la mesure ponctuelle de distance, appliquer un filtrage en exigeant une valeur ni nulle ni infinie (égale à 'inf').

- On peut aussi envisager prendre en compte un plus large éventail de mesures comme par exemple celle des distances entre -5° et +5°, et conserver la plus petite distance mesurée comme celle étant la distance de l'obstacle devant.

- Il serait également pertinent de profiter du lidar réalisant un scan à 360 degrès. Il n'est donc pas nécessaire de tourner le TurtleBot afin de vérifier si l'emplacement est occupé.

- Enfin, à terme, il est souhaitable d'intégrer le code de détection de panneau (défaillant durant la compétition) et celui de suivi de ligne au fonctionnement deu code de parking.
