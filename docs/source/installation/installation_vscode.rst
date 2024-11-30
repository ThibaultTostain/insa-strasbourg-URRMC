Installation et utilisation de VsCode pour écrire des scripts Python
===================================================================

Télécharger VsCode
------------------

Ouvrir un terminal et entrer les commandes :

.. code-block:: bash

   sudo apt update
   sudo apt install curl apt-transport-https

Ajouter le dépôt Microsoft
--------------------------

Ajouter la clé GPG de Microsoft et le dépôt pour installer VsCode :

.. code-block:: bash

   curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo gpg --dearmor -o /usr/share/keyrings/packages.microsoft.gpg

Puis, ajouter le dépôt à la liste des sources de votre système :

.. code-block:: bash

   echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

Installer VsCode
----------------

Mettre à jour les sources des paquets et installer VsCode :

.. code-block:: bash

   sudo apt update
   sudo apt install code

Si tout s'est bien passé, VsCode a été téléchargé. L'application se trouve dans "Show Applications" en bas à gauche sur Ubuntu. 
Il est conseillé de faire un clic droit et de **Ajouter aux favoris** pour y accéder facilement.

Télécharger l'extension Python pour VsCode
------------------------------------------

Il est ensuite recommandé de télécharger l'extension Python pour VsCode. Cela permet d'ajouter une prise en charge complète de Python dans l'éditeur, avec des fonctionnalités comme l'auto-complétion, le débogage et plus encore.
