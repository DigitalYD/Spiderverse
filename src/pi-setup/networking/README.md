For the Raspberry PI that we are using for this project, we are running a PI4 with 4 GB of RAM with Ubuntu Server (64-bit) because of a software that we are using ROS2 (robotic operating system) it is like a API/MQTT mix that can support multiple node send and recieve informations. Raspbian OS that the PI runs off would work but after more research it is not viable based on the forms that we have looked at. We have tested it with the Ubunutu Desktop (64-bit) but it has too much overhead for the PI4 to handle so we switched to the Ubuntu Server.

The ROS2 version that we are going to be using is ROS2: Humble.

Setup:

1. The easiest way to setup to the Internet is with an Ethernet Cable to install all the necessary packages then work on the wireless connection.

2. Yun's PI is setup for this config: Ubuntu Server with Wireless. Thus during any boot up you have to run the "network.sh" that is in this folder to by | bash network.sh | so that it would force the connection to the WIFI
