#!/bin/bash

sudo apt update
sudo apt-get install curl -y
curl -fsSL https://code-server.dev/install.sh | sh
sudo systemctl start code-server@$USER
sudo systemctl enable --now code-server@$USER

sudo apt install nginx -y
sudo systemctl start nginx
sudo systemctl enable nginx

sudo systemctl status nginx