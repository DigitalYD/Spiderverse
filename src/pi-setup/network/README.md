For the Raspberry PI that we are using for this project, we are running a PI4 with 4 GB of RAM with Ubuntu Server (64-bit) because of a software that we are using ROS2 (robotic operating system) it is like a API/MQTT mix that can support multiple node send and recieve informations. Raspbian OS that the PI runs off would work but after more research it is not viable based on the forms that we have looked at. We have tested it with the Ubunutu Desktop (64-bit) but it has too much overhead for the PI4 to handle so we switched to the Ubuntu Server.

The ROS2 version that we are going to be using is ROS2: Humble Hawksbill

Setup:

1. The easiest way to setup to the Internet is with an Ethernet Cable to install all the necessary packages then work on the wireless connection.

2. Yun's PI is setup for this config: Ubuntu Server with Wireless. Thus during any boot up you have to run the "network.sh" that is in this folder to by | bash network.sh | so that it would force the connection to the WIFI

Reference Sites:

https://medium.com/@nullbyte.in/raspberry-pi-4-ubuntu-20-04-lts-ros2-a-step-by-step-guide-to-installing-the-perfect-setup-57c523f9d790
https://wiki.hshl.de/wiki/index.php/Installation_of_Ubuntu_Server_and_ROS2
https://medium.com/spinor/getting-started-with-ros2-install-and-setup-ros2-humble-on-ubuntu-22-04-lts-ad718d4a3ac2
https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line
https://askubuntu.com/questions/1249708/connect-raspberry-pi-4-with-ubuntu-server-to-wifi
https://raspberrypi.stackexchange.com/questions/111722/rpi-4-running-ubuntu-server-20-04-cant-connect-to-wifi
https://www.youtube.com/watch?v=Cw_34fuve6E
https://www.youtube.com/watch?v=P_-_1Ab5jFM