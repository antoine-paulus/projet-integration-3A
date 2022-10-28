Afin de pouvoir lancer le projet, assurez-vous d'avoir installé ROS (version 1), MoveIt! et Gazebo pour le bon fonctionnement du code.

# Installation du projet: 
1) Clonez le dépot.

2) déplacez-vous dans le dossier catkin_ws:
```
cd projet-integration-3A/catkin_ws
```

3) Sourcez ROS (exemple ici avec Noetic)
```
source /opt/ros/noetic/setup.bash
```

4) Lancez la construction du projet:
```
catkin build
```

5) Sourcez de nouveau le projet: 
```
source /devel/setup.bash
```

# Utilisation de la solution
Le paquet de configuration est déjà créé

## Lancer RVIZ uniquement
Pour lancer RVIZ uniquement avec le robot, exécutez cette commande
```
roslaunch hc10_moveit_config demo.launch
```

## Utiliser le programme python
Ouvrez un second terminal, placez vous dans le repository et exécutez cette comande.
```
python3 catkin_ws/src/main.py
```
**NB :** Les consignes pour modifier le code python sont dans le rapport. 

## Utiliser RVIZ et Gazebo
Pour lancer RVIZ et Gazebo avec le robot et l'implémentation du nuage de points, exécutez cette commande
```
roslaunch hc10_movit_config demo_gazebo.launch
```
