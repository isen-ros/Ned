# Sagan
Sagan est pense pour travailler dans un linux avec ROS1 installé.
Malgres ça, si vous allez travailler sur un autre systeme d'exploitation ou sans ROS1 Sagan va marcher partiellement. 

## Installation 
Pour installer sagan il faut une connection internet. 
Ouvrez la terminal bash dans ce dossier et execute

```
	$ ./install-sagan.sh
```  
Cet commande va produire une sortie comme cela 

```
Creating starter scripts pharo and pharo-ui
MetacelloNotification: Fetched -> BaselineOfSagan-CompatibleUserName.1672243663 --- git@github.com:sbragagnolo/Sagan.git[main] --- git@github.com:sbragagnolo/Sagan.git[main]
MetacelloNotification: Loading -> BaselineOfSagan-CompatibleUserName.1672243663 --- git@github.com:sbragagnolo/Sagan.git[main] --- git@github.com:sbragagnolo/Sagan.git[main]
MetacelloNotification: Loaded -> BaselineOfSagan-CompatibleUserName.1672243663 --- git@github.com:sbragagnolo/Sagan.git[main] --- git@github.com:sbragagnolo/Sagan.git[main]
MetacelloNotification: Loading baseline of BaselineOfSagan...
MetacelloNotification: Fetched -> RobotsDuNordROS-CompatibleUserName.1672243678 --- git@github.com:pharo-robotics/RobotsDuNordROS.git[master] --- git@github.com:pharo-robotics/RobotsDuNordROS.git[master]
MetacelloNotification: Fetched -> Sagan-CompatibleUserName.1672243663 --- git@github.com:sbragagnolo/Sagan.git[main] --- git@github.com:sbragagnolo/Sagan.git[main]
MetacelloNotification: Loading -> RobotsDuNordROS-CompatibleUserName.1672243678 ---

```

le script d'installation va aussi creer un script 'sagan.sh' 
En plus pour ouvrir sagan vous allez execute **depuis cet dossier** 

```
	$ ./sagan.sh
``` 

On remarque l'importance de le faire depuis cet dossier. Sinon l'utile ne va pas marcher correctement. 


## Installation windows 
Chez windows on peut installer Sagan aussi, mais il faut le faire depuis un environement [Cygwin](https://www.cygwin.com/).
