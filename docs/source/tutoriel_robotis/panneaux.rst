Detection de panneau
====================

Ce tutoriel vous guidera à travers le processus de reconnaissance et de détection de panneaux pour un robot Turtlebot3.

Préparation
-----------

Avant de commencer, fermez tous les terminaux ouverts précédemment.

Étapes de Configuration
---------------------

1. Lancer l'Autorace
~~~~~~~~~~~~~~~~~~~

Lancez l'environnement Gazebo :

.. code-block:: bash

    roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

2. Téléopération du Robot
~~~~~~~~~~~~~~~~~~~~~~~

Lancez la téléopération par clavier :

.. code-block:: bash

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Positionnement du Robot
^^^^^^^^^^^^^^^^^^^^^^

- Déplacez le robot jusqu'à ce que les panneaux soient clairement visibles par les caméras

Pour cela, cliquez sur le robot dans Gazebo pour le sélectionner, puis utilisez les touches suivantes :
- Touche ``T`` : Mode Translation
  - Permet de déplacer le robot dans le plan (x, y)
  - Cliquez et faites glisser pour repositionner
  
- Touche ``R`` : Mode Rotation
  - Permet de faire pivoter le robot
  - Cliquez et faites tourner pour ajuster l'orientat
- Objectif : Placer les panneaux dans le champ de vision optimal

3. Visualisation de l'Image
~~~~~~~~~~~~~~~~~~~~~~~~~

Lancez la vue d'image :

.. code-block:: bash

    rqt_image_view

Sélectionnez le flux ``/camera/image_compensated``

4. Capture des Panneaux
~~~~~~~~~~~~~~~~~~~~~

Captures d'Images
^^^^^^^^^^^^^^^^^

- Utilisez ``rqt_image_view`` pour capturer les images des panneaux
- Techniques de capture :
  * Faites une capture d'écran
  * Rogner l'image pour ne garder que le panneau

Nommage des Fichiers
^^^^^^^^^^^^^^^^^^^

Sauvegardez les images dans ``/turtlebot3_autorace_2020/turtlebot3_autorace_detect/image/``

Conventions de Nommage :
- ``construction.png``
- ``intersection.png``
- ``left.png``
- ``right.png``
- ``parking.png``
- ``stop.png``
- ``tunnel.png``

5. Calibration Intrinsèque
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

6. Calibration Extrinsèque
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

7. Détection de Panneau
~~~~~~~~~~~~~~~~~~~~~

Lancement de la Détection
^^^^^^^^^^^^^^^^^^^^^^^^

Remplacez ``SELECT_MISSION`` par la mission appropriée :

- ``intersection``
- ``construction``
- ``parking``
- ``level_crossing``

.. code-block:: bash

    roslaunch turtlebot3_autorace_detect detect_sign.launch mission:=SELECT_MISSION

8. Visualisation de la Détection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Lancez à nouveau ``rqt_image_view``

Sélectionnez le flux ``/detect/image_traffic_sign/compressed``

Types de Missions
^^^^^^^^^^^^^^^

1. Mission Intersection
   - Panneaux de direction
   - Gestion des carrefours

2. Mission Construction
   - Signalisation de chantier
   - Navigation dans des zones en travaux

Conseils Avancés
---------------

- Assurez-vous d'une bonne luminosité
- Capturez des images nettes et contrastées
- Vérifiez la qualité de détection
- Ajustez les paramètres si nécessaire

Dépannage
---------

- Problèmes de détection ?
  * Vérifiez la qualité des images capturées
  * Assurez-vous que les panneaux sont bien éclairés
  * Repositionnez le robot si nécessaire