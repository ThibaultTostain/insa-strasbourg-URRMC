Suivi de ligne et arrêt au damier 
===============================================
Cette partie correspond au code permettant de réaliser la première partie de la qualification.
----------------------------------------------------------------------------------------------
Le robot doit être capable de suivre la ligne sur une certaine distance et de s'arrêter uniquement lorsqu'il rencontre un damier. 

Pour effectuer cela, nous aurons besoin du code de détection du damier et du code de suivi de ligne. A l'aide d'un fichier .launch, nous pourrons exécuter les deux codes ensembles. Enfin, nous utiliserons également un fichier master_node qui permet de

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

