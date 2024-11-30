Programmation Python avec ROS1 Noetic
====================================

Introduction
------------

Ce tutoriel vous guidera à travers l'utilisation de Python avec ROS1 Noetic, en couvrant la création de packages, la compilation, et les concepts de base de la programmation ROS.

Configuration de l'Environnement
-------------------------------

Prérequis
~~~~~~~~~

- ROS Noetic installé
- Python 3.x
- Catkin workspace configuré

Création d'un Workspace Catkin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

Configuration du Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^

Ajoutez la source du workspace à votre fichier ``.bashrc`` :

.. code-block:: bash

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Création de Packages ROS
-----------------------

Méthode de Création
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    cd ~/catkin_ws/src
    catkin_create_pkg mon_package rospy std_msgs

Structure d'un Package ROS
^^^^^^^^^^^^^^^^^^^^^^^^^

::

    mon_package/
    ├── CMakeLists.txt
    ├── package.xml
    └── src/
        └── scripts/

Éditeur de Code Recommandé
------------------------

Visual Studio Code (VSCode)
~~~~~~~~~~~~~~~~~~~~~~~~~~

Avantages :
- Support ROS intégré
- Extensions Python
- Débogage intégré
- Auto-complétion

Extensions VSCode recommandées :
- "ROS" 
- "Python"
- "CMake"
- "CMake Tools"

Configuration de CMakeLists.txt
------------------------------

Structure de Base
~~~~~~~~~~~~~~~~

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.0.2)
    project(mon_package)

    ## Compiler options
    find_package(catkin REQUIRED COMPONENTS
      rospy
      std_msgs
    )

    ## Declare a Python script as an executable
    catkin_install_python(PROGRAMS 
      src/scripts/mon_script.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

Notes Importantes
^^^^^^^^^^^^^^^^

- Chaque nouveau fichier Python doit être ajouté dans ``PROGRAMS``
- Rendez les scripts exécutables : ``chmod +x mon_script.py``

Exemple de Code ROS : Publisher
------------------------------

Création d'un Publisher
~~~~~~~~~~~~~~~~~~~~~~

Fichier ``publisher.py`` :

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from std_msgs.msg import String

    def talker():
        # Initialisation du nœud
        rospy.init_node('talker', anonymous=True)
        
        # Création d'un publisher sur le topic 'chatter'
        pub = rospy.Publisher('chatter', String, queue_size=10)
        
        # Fréquence de publication
        rate = rospy.Rate(10)  # 10hz
        
        # Boucle de publication
        while not rospy.is_shutdown():
            hello_str = f"Bonjour ROS {rospy.get_time()}"
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

    if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException:
            pass

Exemple de Code ROS : Subscriber
-------------------------------

Fichier ``subscriber.py`` :

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from std_msgs.msg import String

    def callback(data):
        """
        Fonction de callback appelée à chaque réception de message
        """
        rospy.loginfo(rospy.get_caller_id() + 
                      f" J'ai reçu : {data.data}")

    def listener():
        # Initialisation du nœud
        rospy.init_node('listener', anonymous=True)
        
        # Abonnement au topic 'chatter'
        rospy.Subscriber('chatter', String, callback)
        
        # Spin() empêche le programme de quitter
        rospy.spin()

    if __name__ == '__main__':
        listener()

Compilation et Exécution
-----------------------

Compiler le Workspace
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    cd ~/catkin_ws
    catkin_make

Exécution des Nœuds
~~~~~~~~~~~~~~~~~~

Terminal 1 (Master) :

.. code-block:: bash

    roscore

Terminal 2 (Publisher) :

.. code-block:: bash

    rosrun mon_package publisher.py

Terminal 3 (Subscriber) :

.. code-block:: bash

    rosrun mon_package subscriber.py

Fichiers Launch : Automatisation du Démarrage des Nœuds
-----------------------------------------------------

Introduction aux Fichiers Launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Les fichiers launch sont des fichiers XML qui permettent de :
- Démarrer plusieurs nœuds simultanément
- Configurer des paramètres globaux
- Définir des arguments
- Simplifier le lancement de systèmes complexes

Structure de Base d'un Fichier Launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Créez un fichier ``mon_noeud.launch`` dans le dossier ``launch/`` de votre package :

.. code-block:: xml

    <launch>
        <!-- Démarrage de plusieurs nœuds -->
        <node name="talker" pkg="mon_package" type="publisher.py" output="screen"/>
        <node name="listener" pkg="mon_package" type="subscriber.py" output="screen"/>
        
        <!-- Configuration de paramètres -->
        <param name="mon_parametre" value="valeur_par_defaut"/>
        
        <!-- Groupe de nœuds -->
        <group ns="mon_namespace">
            <node name="noeud_specifique" pkg="mon_package" type="mon_script.py"/>
        </group>
    </launch>

Attributs Principaux des Nœuds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- ``name`` : Nom unique du nœud
- ``pkg`` : Package contenant le script
- ``type`` : Nom du script exécutable
- ``output="screen"`` : Affiche les logs dans le terminal
- ``respawn="true"`` : Redémarre le nœud s'il plante
- ``required="true"`` : Arrête roscore si le nœud s'arrête

Exemple Avancé de Launch
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: xml

    <launch>
        <!-- Arguments avec valeurs par défaut -->
        <arg name="robot_name" default="turtlebot3"/>
        
        <!-- Utilisation de variables -->
        <param name="robot_description" 
               command="$(find xacro)/xacro '$(find turtlebot3_description)/urdf/turtlebot3_burger.urdf.xacro'" />
        
        <!-- Nœuds avec arguments -->
        <node name="robot_state_publisher" pkg="robot_state_publisher" 
              type="robot_state_publisher">
            <param name="robot_description" 
                   command="$(find xacro)/xacro '$(find turtlebot3_description)/urdf/turtlebot3_burger.urdf.xacro'"/>
        </node>
        
        <!-- Remapping de topics -->
        <node name="my_node" pkg="mon_package" type="mon_script.py">
            <remap from="input_topic" to="modified_input"/>
            <remap from="output_topic" to="modified_output"/>
        </node>
        
        <!-- Inclusion conditionnelle -->
        <include if="$(arg debug_mode)" 
                 file="$(find mon_package)/launch/debug.launch"/>
    </launch>

Types d'Arguments Launch
~~~~~~~~~~~~~~~~~~~~~~

1. Arguments Simples
^^^^^^^^^^^^^^^^^^^

.. code-block:: xml

    <arg name="mon_argument" default="valeur_defaut"/>
    <arg name="mon_argument" value="valeur_fixe"/>

2. Arguments Booléens
^^^^^^^^^^^^^^^^^^^^

.. code-block:: xml

    <arg name="debug_mode" default="false"/>
    <node if="$(arg debug_mode)" ... />

3. Passage d'Arguments en Ligne de Commande
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    roslaunch mon_package mon_fichier.launch mon_argument:=ma_valeur

