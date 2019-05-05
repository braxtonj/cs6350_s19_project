#!/usr/bin/env bash

# ================================================================= #
#                                                                   #
#   Installs dependencies for ll4ma_robots_gazebo using wstool.     #
#                                                                   #
# ----------------------------------------------------------------- #
#  USAGE                                                            #
# ----------------------------------------------------------------- #
#   Usage if wstool is already installed:                           #
#     $ ./install_dependencies.sh                                   #
#                                                                   #
#   Usage if wstool is NOT installed and you have sudo privileges:  #
#     $ sudo ./install_dependencies.sh                              #
#                                                                   #
# ----------------------------------------------------------------- #
#  OPTIONS                                                          #
# ----------------------------------------------------------------- #
#   BAXTER : set true if you want to simulate Baxter robot          #
#                                                                   #
# ================================================================= #

BAXTER=false

#-------------------------------------------------------------------#

# install system dependencies if you ran with sudo and have privileges
if [ "$EUID" = 0 ]; then
    apt install python-wstool
fi

# run wstool
SRC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd $SRC_DIR
if [ ! -f $SRC_DIR/.rosinstall ]; then
    wstool init
fi

# switch on whether to use SSH key version or https for Git
RESULT=$(ssh git@bitbucket.org 2>&1)
if [[ "$RESULT" =~ "denied" ]]; then
    # SSH not setup, use HTTPS
    wstool merge $SRC_DIR/ll4ma_robots_gazebo/.ll4ma_robots_gazebo.http.rosinstall
else
    # Use SSH keys
    wstool merge $SRC_DIR/ll4ma_robots_gazebo/.ll4ma_robots_gazebo.ssh.rosinstall
fi


if $BAXTER; then
    wstool merge $SRC_DIR/ll4ma_robots_gazebo/.baxter_depends.rosinstall
fi
wstool update

