#!/bin/bash
#
# ArUco Materials Installation Script for Gazebo Classic 11
# 
# This script installs ArUco marker textures and material scripts to
# Gazebo's system-wide media path. Required for spawn_4_bots_aruco.launch.py
#
# Usage: ./install_aruco_materials.sh
#
# Author: Copilot
# Date: January 29, 2026
# Repository: https://github.com/gaurav/ros2_ws (update with your repo)
#

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ArUco Materials Installation Script  ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source paths (in ros2_ws)
SOURCE_MATERIAL="${SCRIPT_DIR}/src/turtlebot3/turtlebot3_description/media/materials/scripts/aruco.material"
SOURCE_TEXTURES="${SCRIPT_DIR}/src/turtlebot3/turtlebot3_description/media/materials/textures"

# Destination paths (Gazebo system-wide)
GAZEBO_SCRIPTS="/usr/share/gazebo-11/media/materials/scripts"
GAZEBO_TEXTURES="/usr/share/gazebo-11/media/materials/textures"

# Check if source files exist
echo -e "${YELLOW}Checking source files...${NC}"

if [ ! -f "$SOURCE_MATERIAL" ]; then
    echo -e "${RED}ERROR: Material file not found at:${NC}"
    echo "  $SOURCE_MATERIAL"
    exit 1
fi

TEXTURE_COUNT=$(ls -1 ${SOURCE_TEXTURES}/agent_*.png 2>/dev/null | wc -l)
if [ "$TEXTURE_COUNT" -eq 0 ]; then
    echo -e "${RED}ERROR: No texture files (agent_*.png) found at:${NC}"
    echo "  $SOURCE_TEXTURES"
    exit 1
fi

echo -e "${GREEN}✓ Found material file: aruco.material${NC}"
echo -e "${GREEN}✓ Found $TEXTURE_COUNT texture files${NC}"
echo ""

# Check if Gazebo directories exist
if [ ! -d "$GAZEBO_SCRIPTS" ]; then
    echo -e "${RED}ERROR: Gazebo scripts directory not found:${NC}"
    echo "  $GAZEBO_SCRIPTS"
    echo "  Is Gazebo 11 installed?"
    exit 1
fi

if [ ! -d "$GAZEBO_TEXTURES" ]; then
    echo -e "${RED}ERROR: Gazebo textures directory not found:${NC}"
    echo "  $GAZEBO_TEXTURES"
    exit 1
fi

echo -e "${GREEN}✓ Gazebo 11 media directories found${NC}"
echo ""

# Install files (requires sudo)
echo -e "${YELLOW}Installing files (requires sudo)...${NC}"
echo ""

# Copy material file
echo "  Copying aruco.material..."
sudo cp "$SOURCE_MATERIAL" "$GAZEBO_SCRIPTS/"

# Copy texture files
echo "  Copying texture files..."
sudo cp ${SOURCE_TEXTURES}/agent_*.png "$GAZEBO_TEXTURES/"

# Fix permissions
echo "  Setting permissions (chmod 644)..."
sudo chmod 644 "${GAZEBO_SCRIPTS}/aruco.material"
sudo chmod 644 ${GAZEBO_TEXTURES}/agent_*.png

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Installation Complete!               ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Installed files:"
echo "  - ${GAZEBO_SCRIPTS}/aruco.material"
for f in ${GAZEBO_TEXTURES}/agent_*.png; do
    echo "  - $f"
done
echo ""
echo -e "${GREEN}You can now run:${NC}"
echo "  ros2 launch turtlebot3_gazebo spawn_4_bots_aruco.launch.py"
echo ""
