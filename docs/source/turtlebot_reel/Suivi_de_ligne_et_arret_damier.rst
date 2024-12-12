Suivi de ligne et arrêt au damier 
===============================================
Cette partie correspond au code permettant de réaliser la première partie de la qualification.
----------------------------------------------------------------------------------------------
Le robot doit être capable de suivre la ligne sur une certaine distance et de s'arrêter uniquement lorsqu'il rencontre un damier. 

Pour effectuer cela, nous aurons besoin du code de détection du damier et du code de suivi de ligne. A l'aide d'un fichier .launch, nous pourrons exécuter les deux codes ensembles. Enfin, nous utiliserons également un fichier master_node qui permet de contrôler l'état d'une commande (activer ou désactiver) en utilisant la touche espace.


1 . Code de détection du damier 
-----------------------------

Le but principal de ce code est de configurer et exécuter un noeud ROS quoi détecte un motif de damier dans les images reçues de la caméra. Lorsque le damier est détecté, il attend 15 secondes avant d'envoyer une commande d'arrêt.

.. code-block:: bash

    #!/usr/bin/env python 
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import UInt8
    from cv_bridge import CvBridge
    import numpy as np
    import rospkg
    import cv2
    import rospy
    
    class StopAtDamier:
        def __init__(self):
            rospy.init_node('stop_at_damier')
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('/camera/image_projected/compressed', CompressedImage, self.image_callback)
            self.command_pub = rospy.Publisher('/command', UInt8, queue_size=10)
            
            # Flag pour gérer l'état de détection
            self.damier_detected = False
            self.command = UInt8()
            
            
            #Pour voir si on est bien dans le fichier
            rospy.loginfo("\n\rNoeud du damier en attente\n")
    
            # Utiliser rospkg pour obtenir le chemin absolu de l'image
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('competition')  
            image_path = package_path + '/pictures/damier.png'
    
            # Charger l'image de référence du damier
            try:
                self.reference_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
                if self.reference_image is None:
                    rospy.logwarn(f"\nErreur lors du chargement de l'image de référence à {image_path}\n")
            except Exception as e:
                rospy.logerr(f"\nErreur lors du chargement de l'image de référence : {e}\n")
                self.reference_image = None
    
        def image_callback(self, data):
            try:
                np_arr = np.frombuffer(data.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
            except Exception as e:
                print(f"\nErreur lors du traitement de l'image : {e}\n")
                return
    
            if self.reference_image is not None:
                # Redimensionner l'image de référence à la taille de l'image reçue
                ref_image = cv2.resize(self.reference_image, (cv_image.shape[1], cv_image.shape[0]))
    
                # Comparer l'image du topic avec l'image de référence
                result = cv2.matchTemplate(cv_image, ref_image, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
                # Si le score de correspondance est élevé, on détecte un damier
                if max_val > 0.1:       #valeur de base 0.6
                    if not self.damier_detected:
                        rospy.loginfo("\nDamier détecté, attente avant arrêt\n")
                        self.damier_detected = True
                        rospy.sleep(15)  # Délai en secondes
                        self.publish_command(0)  # Publie le signal d'arrêt après le délai
    
        def publish_command(self, value):
            self.command.data = value
            self.command_pub.publish(self.command)
    
        def run(self):
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                rate.sleep()
    
    if __name__ == '__main__':
        try:
            node = StopAtDamier()
            node.run()
        except rospy.ROSInterruptException:
            pass

Explication rapide du code : 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
La première partie correspond aux différentes importations des modules/librairies nécessaires pour la suite du code. 

La fonction "init" permet : 
    - D'initialiser un noeud ROS nommé "stop_at_damier" 
    - De s'abbonner au topic "/camera/image_projected/compressed" pour recevoir les images compressées
    - De publier sur le topic "/command" pour envoyé des commandes 
    - Charger l'image de référence avec self.reference_image

La fonction "image_callback" : 
    Elle est appelée chaque fois qu'une nouvelle image est reçue. Elle permet de traiter les image reçues, de les comparer avec l'image de référence du damier et si le damier est détecté, de publier une commande pour arrêter le robot après un délais de 15 secondes.

La fonction "publish_command" :
    Elle permet de publier une commande avec la valeur spécifiée sur le topic "/command"

La fonction "run":
    Correspond à la boucle principale qui maintient le noeud en vie et vérifie si ROS doit s'arrêter.

Enfin, la dernière partie du code permet le lancement du noeud.

2 . Code du suivi de ligne  
-----------------------------

Concernant le code de suivi de ligne, on utilise celui du tutoriel ROBOTIS, que l'on retrouve sur le github au lien suivant : https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020/tree/main . 

Les codes que nous utiliserons sont les .launch suivants : 
    - turtlebot3_autorace_detect/launch/detect_lane.launch
    - turtlebot3_autorace_driving/launch/turtlebot3_autorace_control_lane.launch

Ces codes permettent d'exécuter les codes Python suivants : 
    - turtlebot3_autorace_detect/nodes/detect_lane
    - turtlebot3_autorace_driving/nodes/control_lane

3 . Code "master_node"
-----------------------------

Le but principal de ce code est de définir un noeud nommé master_node qui permet de contrôler l'état d'un commande en utilisant la touche espace. 


.. code-block:: bash
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    # Author: PALISSE Volia, WAECHTER Thibaut, YOUBI Lounès, OLIVEIRA Théo
    
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
            print("\n\r")
            rospy.loginfo(f"État initial de la commande: {self.command_state}\n")
    
        def command_callback(self, command_msg):
            self.command_state = command_msg.data
            print("\n\r")
            rospy.loginfo(f"Nouvelle valeur de commande reçue: {self.command_state}\n")
    
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
            rospy.loginfo("Appuyez sur ESPACE pour alterner l'état du topic /command entre 0 et 1\n")
            
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == ' ':
                    self.command_state = 1 if self.command_state == 0 else 0
                    print("\n\r")
                    rospy.loginfo(f"Command: {self.command_state}\n")
                    self.pub_command.publish(self.command_state)
                elif key == '\x03':  # touche Ctrl+C
                    print("\n\r")
                    rospy.loginfo("Arrêt du robot.\n")
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                    print("\n\r")
                    rospy.signal_shutdown("Arrêt demandé par l'utilisateur\n")
                    break
    
    def main():
        rospy.init_node('master_node')
        
        try:
            node = MasterNode()
            node.run()
        except rospy.ROSInterruptException:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        except Exception as e:
            print("\r")
            rospy.logerr(f"\rErreur: {str(e)}\n")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin.fileno()))
    
    if __name__ == '__main__':
        main()

Explication rapide du code : 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 

Dans ce code on retrouve la fonction 'init' qui permet d'initialiser le noeud et le publisher et subscriber permettant de publier ou écouter des messages sur le topic "/command". On a également une variable pour stocket l'état de la commande qui est à 0 si désactivé et 1 si activé.

Ensuite, la fonction 'command_callback' permet de mettre à jour l'état de la commande avec la valeur reçue sur le topic.

La fonction 'get_key' a pour but de gérer les entrées du clavier. 

Enfin, la fonction 'run' permet d'écouter les entrées clavier et de publier les commandes en conséquences.

Finalement la fonction 'main' initialise le noeud, crée une instance de MasterNode et appelle la méthode 'run'.

La fin du code permet l'exécution du main.


4 . Fichier .launch
-----------------------------

Le fichier .launch en ROS est un fichier qui sert à démarrer et configurer plusieurs noeuds et paramètres de ROS en une seule commande. En clair, ce fichier va nous permettre ici de lancer l'ensemble des codes exposés précédemment qui sont nécessaires pour faire la mission suivi de ligne et arrêt au damier.

Ici, grâce à ce fichier, on lance en même temps les noeuds suivants : 
    - calibration intrinsèque de la caméra 
    - Calibration extrinsèque de la caméra
    - Lane detection
    - Master Node
    - Stop at damier
    - Control Lane

.. code-block:: bash
    
    <launch>
        <arg name="mode" default="action"/>
        
        <!-- Lancer la calibration intrinsèque de la caméra -->
        <include file='$(find turtlebot3_autorace_camera)/launch/intrinsic_camera_calibration.launch' />
    
        <!-- Lancer la calibration extrinsèque de la caméra -->
        <include file='$(find turtlebot3_autorace_camera)/launch/extrinsic_camera_calibration.launch' />
    
        <!-- Lancement du lane detection -->
        <include file='$(find turtlebot3_autorace_detect)/launch/detect_lane.launch' />
    
        <!-- Lancement du Master node avec gestion de l'arrêt -->
        <node pkg="competition" name="master_node" type="master_node.py" output="screen" required="false" respawn="false">
        </node>
        
        <!-- Lancement du noeud pour l'arret sur le damier-->
        <node pkg="competition" type="stop_at_damier.py" name="stop_at_damier" output="screen">
            <remap from="/camera/image_raw" to="/camera/image_raw" />
            <remap from="/cmd_vel" to="/cmd_vel" />
            <remap from="/command" to="/command" />
        </node>
        
        <!-- Lancement du lane control -->
        <include file='$(find turtlebot3_autorace_driving)/launch/turtlebot3_autorace_control_lane.launch' />
      
    </launch>



