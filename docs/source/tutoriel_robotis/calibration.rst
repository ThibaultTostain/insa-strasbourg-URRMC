Calibration camera
==================

Ce tutoriel vous guidera à travers le processus de calibration de caméra pour un robot Turtlebot3 destiné au suivi de ligne.

Prérequis
---------
- Robot Turtlebot3
- ROS Noetic installé
- Gazebo
- Package turtlebot3_autorace

Étapes de Calibration
--------------------

1. Lancer Gazebo
~~~~~~~~~~~~~~~~

Ouvrez un terminal et exécutez la commande suivante pour lancer l'environnement Gazebo :

.. code-block:: bash

    roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

2. Calibration Intrinsèque
~~~~~~~~~~~~~~~~~~~~~~~~~

La calibration intrinsèque vise à comprendre les caractéristiques internes de la caméra qui affectent la façon dont les images sont projetées. Elle permet de déterminer :

- Les coefficients de distorsion
- La distance focale
- Le point principal (centre optique)

Objectifs principaux :
- Corriger les déformations optiques
- Compenser les aberrations géométriques
- Établir une correspondance précise entre les pixels et les coordonnées réelles

Dans un nouveau terminal, lancez la calibration intrinsèque :

.. code-block:: bash

    roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

3. Calibration Extrinsèque
~~~~~~~~~~~~~~~~~~~~~~~~~

La calibration extrinsèque détermine la position et l'orientation de la caméra par rapport à un référentiel global. Elle permet de :

- Calculer la transformation spatiale entre le repère de la caméra et le repère du robot
- Aligner précisément le champ de vision de la caméra
- Compenser les décalages physiques de montage

Objectifs :
- Garantir une perception correcte de l'environnement
- Améliorer la précision de la navigation et du suivi de ligne
- Compenser les imperfections de montage mécanique

Ouvrez un autre terminal et lancez la calibration extrinsèque :

.. code-block:: bash

    roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration

4. Lancer rqt
~~~~~~~~~~~~~

Exécutez rqt dans un nouveau terminal :

.. code-block:: bash

    rqt

5. Configuration des Fenêtres d'Image
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Sélectionnez ``Plugins > Visualisation > Vue Image``
- Créez deux fenêtres d'affichage d'images
- Affichez les flux suivants :
  * ``/camera/image_extrinsic_calib/compressed``
  * ``/camera/image_projected_compensated``

.. image:: pictures/noetic_extrinsic_calibration.png
   :alt: Capture d'écran de Terminator
   :width: 600
   :align: center

6. Reconfiguration
~~~~~~~~~~~~~~~~~

Ouvrez un nouveau terminal et lancez rqt_reconfigure :

.. code-block:: bash

    rosrun rqt_reconfigure rqt_reconfigure

7. Ajustement des Paramètres
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Dans rqt_reconfigure, ajustez les paramètres suivants :
- ``/camera/image_projection``
- ``/camera/image_compensation_projection``

.. image:: pictures/noetic_extrinsic_calibration_reconfigure.png
   :alt: Capture d'écran de Terminator
   :width: 600
   :align: center

8. Enregistrement des Paramètres
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Après les ajustements, écrasez les valeurs dans les fichiers YAML suivants :

- ``Turtlebot3_autorace_camera/calibration/extrinsic_calibration/compensation.yaml``
- ``Turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml``

.. image:: pictures/noetic_projection_yaml.png
   :alt: Capture d'écran de Terminator
   :width: 600
   :align: center

.. image:: pictures/noetic_compensation_yaml.png
   :alt: Capture d'écran de Terminator
   :width: 600
   :align: center


Conseils Importants
------------------

- Soyez précis lors de l'ajustement des paramètres
- Vérifiez visuellement l'alignement et la compensation de l'image
- Enregistrez vos configurations pour une utilisation future

