#!/bin/bash

# Exit on any error
set -e

echo "Starting Docker installation for Raspberry Pi 4..."

# Update system
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install dependencies
echo "Installing required dependencies..."
sudo apt install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker's GPG key
echo "Adding Docker's official GPG key..."
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up Docker repository
echo "Setting up Docker repository..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update package index
echo "Updating package index with Docker repository..."
sudo apt update

# Install Docker
echo "Installing Docker..."
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add user to docker group
echo "Adding current user to docker group..."
sudo usermod -aG docker $USER

# Enable and start Docker
echo "Enabling and starting Docker service..."
sudo systemctl enable docker
sudo systemctl start docker

# Verify installation
echo "Verifying Docker installation..."
docker --version

# Test Docker
echo "Testing Docker with hello-world container..."
docker run hello-world

echo "Docker installation complete!"
echo "NOTE: You may need to log out and log back in for group changes to take effect."
echo "To verify Docker is working after logging back in, run: docker run hello-world"
