Détection de Ligne Gazebo
=========================

Ce tutoriel vous guidera à travers le processus de détection et de suivi de ligne pour un robot Turtlebot3.

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

2. Placement et Déplacement du Robot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Positionnement du Robot entre les Lignes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pour placer précisément le robot entre les lignes, vous disposez de plusieurs méthodes :

Téléopération Clavier
""""""""""""""""""""

Utilisez la téléopération par clavier :

.. code-block:: bash

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Raccourcis de Manipulation dans Gazebo
""""""""""""""""""""""""""""""""""""""

Une fois le robot sélectionné dans Gazebo, utilisez ces raccourcis :

- Touche ``T`` : Mode Translation
  
  - Permet de déplacer le robot dans le plan (x, y)
  - Cliquez et faites glisser pour repositionner
  
- Touche ``R`` : Mode Rotation
  
  - Permet de faire pivoter le robot
  - Cliquez et faites tourner pour ajuster l'orientation

Conseils :

- Utilisez ces raccourcis pour un positionnement précis
- Visez un alignement optimal avec les lignes de la piste

1. Calibration Intrinsèque
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

4. Calibration Extrinsèque
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

5. Lancement de la Détection de Ligne
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration

6. Lancer rqt
~~~~~~~~~~~~~

.. code-block:: bash

    rqt

7. Configuration de l'Image
~~~~~~~~~~~~~~~~~~~~~~~~~~

- Lancez Image View via ``Plugins > Visualisation > Image view``
- Affichez les trois images de détection :
  
  * Image de détection générale
  * Détection de ligne jaune
  * Détection de ligne blanche

.. image:: pictures/install_terminator.png
   :alt: Capture d'écran de Terminator
   :width: 600
   :align: center

.. image:: pictures/noetic_detect_image_lane.png
   :alt: img lane
   :width: 600
   :align: center

.. image:: pictures/noetic_detect_white_lane.png
   :alt: white lane
   :width: 600
   :align: center

.. image:: pictures/noetic_detect_yellow_lane.png
   :alt: yellow lane
   :width: 600
   :align: center

1. Reconfiguration
~~~~~~~~~~~~~~~~~

.. code-block:: bash

    rosrun rqt_reconfigure rqt_reconfigure

9. Ajustement des Paramètres de Détection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Dans ``rqt_reconfigure``, sélectionnez ``detect_lane`` et ajustez les paramètres de filtrage des couleurs.

.. image:: pictures/noetic_detect_reconfigure_lane.png
   :alt: reconfigure lane
   :width: 600
   :align: center

Conseils de Calibration des Couleurs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Processus de Calibration HSV
"""""""""""""""""""""""""""

1. Teinte (Hue)
   
   - Représente la couleur de base
   - Chaque couleur (jaune, blanc) a sa propre plage
   - Commencez par des valeurs basses à élevées

2. Saturation
   
   - Définit l'intensité de la couleur
   - Calibrez de faible à élevé
   - Chaque couleur a sa propre plage de saturation

3. Luminosité (Value)
   
   - Contrôle la brillance
   - La valeur basse n'affecte pas la sortie
   - Ajustez principalement la valeur haute
   - Visez une détection optimale

4.  Enregistrement des Paramètres
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ouvrez ``lane.yaml`` dans ``turtlebot3_autorace_detect/param/lane/`` et enregistrez les nouvelles valeurs.

.. image:: pictures/noetic_lane_lane.png
   :alt: yaml lane
   :width: 600
   :align: center

11. Finalisation
~~~~~~~~~~~~~~~

- Fermez ``rqt_reconfigure`` et le nœud ``detect_lane``
- Relancez la détection :

.. code-block:: bash

    roslaunch turtlebot3_autorace_detect detect_lane.launch

12. Contrôle de Lane
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch

Dépannage
---------

- Vérifiez l'éclairage de l'environnement
- Ajustez finement les paramètres HSV
- Assurez-vous que le robot est bien positionné