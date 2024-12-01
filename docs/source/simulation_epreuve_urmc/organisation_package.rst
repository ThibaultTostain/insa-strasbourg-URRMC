Package Tournoi URMC
====================

Présentation Générale
--------------------

Cette page est conçue pour décrire l'organisation du package tournoi, permettant de gérer plusieurs missions de manière indépendante et évolutive.

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

Initialisez le package dans votre workspace catkin :

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

Exemple : 

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

Le ``master_node`` est un composant central qui contrôle le démarrage des missions.

Exemple de Code Master Node
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    # Author: PALISSE Volia, WAECHTER Thibaut, YOUBI Lounès

    import rospy
    from std_msgs.msg import UInt8
    import sys
    import select
    import termios
    import tty
    import signal

    class MasterNode:
        def __init__(self):
            # Initialisation du publisher et subscriber
            self.pub_command = rospy.Publisher('/command', UInt8, queue_size=1)
            self.sub_command = rospy.Subscriber('/command', UInt8, self.command_callback, queue_size=1)
            self.command_state = 0  # 0 = désactivé, 1 = activé
            self.settings = termios.tcgetattr(sys.stdin)
            
            rospy.loginfo(f"\nÉtat initial de la commande: {self.command_state}")

        def command_callback(self, command_msg):
            self.command_state = command_msg.data
            rospy.loginfo(f"\nNouvelle valeur de commande reçue: {self.command_state}")

        def get_key(self):
            try:
                tty.setraw(sys.stdin.fileno())
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                else:
                    key = ''
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key

        def run(self):
            rospy.loginfo("\nAppuyez sur ESPACE pour alterner l'état du topic /command entre 0 et 1")
            
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == ' ':
                    self.command_state = 1 if self.command_state == 0 else 0
                    rospy.loginfo(f"Command: {self.command_state}")
                    self.pub_command.publish(self.command_state)
                elif key == '\x03':  # Ctrl+C
                    rospy.loginfo("\nArrêt du robot.")
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                    rospy.signal_shutdown("\nArrêt demandé par l'utilisateur")
                    break

    def main():
        rospy.init_node('master_node')
        
        try:
            node = MasterNode()
            node.run()
        except rospy.ROSInterruptException:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        except Exception as e:
            rospy.logerr(f"Erreur: {str(e)}")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin.fileno()))

    if __name__ == '__main__':
        main()

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

