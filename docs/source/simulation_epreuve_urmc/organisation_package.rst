Package Tournoi pour Turtlebot
==============================

Présentation Générale
--------------------

Ce package est conçu pour organiser un tournoi de Turtlebot avec une architecture modulaire et flexible, permettant de gérer plusieurs missions de manière indépendante et évolutive.

Structure du Package
-------------------

L'arborescence complète du package ``tournoi`` se présente comme suit :

.. code-block:: bash

    tournoi/
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   ├── mission1.launch
    │   ├── mission2.launch
    │   ├── mission3.launch
    │   ├── mission4.launch
    │   └── mission5.launch
    └── scripts/
        ├── mission_1/
        │   ├── __init__.py
        │   ├── master_node.py
        │   └── other_nodes.py
        ├── mission_2/
        │   ├── __init__.py
        │   ├── master_node.py
        │   └── other_nodes.py
        # ... (autres missions similaires)

Création du Package
------------------

Pour initialiser le package dans votre workspace catkin :

.. code-block:: bash

    cd ~/catkin_ws/src
    catkin_create_pkg tournoi rospy std_msgs

Explication de l'Arborescence
----------------------------

1. Dossier ``launch/``
^^^^^^^^^^^^^^^^^^^^

- Contient les fichiers de lancement pour chaque mission
- Permet de configurer et démarrer les nœuds spécifiques à chaque mission

Exemple de fichier ``mission1.launch`` :

.. code-block:: xml

    <launch>
        <node name="master_node" pkg="tournoi" 
              type="mission_1/master_node.py" output="screen"/>
        <node name="mission_node" pkg="tournoi" 
              type="mission_1/other_nodes.py" output="screen"/>
    </launch>

2. Dossier ``scripts/``
^^^^^^^^^^^^^^^^^^^^^

- Organisation modulaire par mission
- Chaque mission possède ses propres nœuds
- Permet une évolution indépendante des missions

Stratégie de Développement
-------------------------

Approche de Réplication des Nœuds
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Contrairement aux pratiques classiques de ROS, nous avons choisi de répliquer certains nœuds entre les missions afin de maintenir la stabilité du code existant, permettre des modifications indépendantes pour chaque mission, et éviter les régressions lors de l'ajout de nouvelles missions.
.. code-block:: bash

    scripts/
    ├── mission_1/
    │   ├── suivi_de_ligne.py
    │   └── master_node_1.py
    ├── mission_2/
    │   ├── suivi_de_ligne_2.py
    │   └── master_node_2.py

Le Nœud Master
--------------

Rôle Principal
^^^^^^^^^^^^^

Le ``master_node`` est un composant central qui contrôle le démarrage des missions

Exemple de Code Master Node
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from std_msgs.msg import Int8
    import sys
    import termios
    import tty

    def master_node():
        pub = rospy.Publisher('/command', Int8, queue_size=10)
        rospy.init_node('master_node', anonymous=True)
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Logique de publication de commandes
            # Par exemple, publication alternée de 0 et 1
            command = Int8(data=(0 if pub.get_num_connections() % 2 == 0 else 1))
            pub.publish(command)
            rate.sleep()

Conseils Pratiques
-----------------

- Rendez les scripts exécutables : ``chmod +x``
- Compilez : ``catkin_make``
- Sourcez votre workspace : ``source ~/catkin_ws/devel/setup.bash``

Avantages de l'Architecture
--------------------------

- **Modularité** maximale
- **Contrôle centralisé** via le master_node
- **Flexibilité** d'exécution
- **Facilité de débogage**
- **Extensibilité** pour de futures missions

Conclusion
----------

Cette architecture offre une approche robuste et flexible pour développer un tournoi de Turtlebot, en permettant une évolution contrôlée et indépendante de chaque mission.