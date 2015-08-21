# *************************************************
# drone_install.sh - Set up Ubuntu for the AR Drone
# *************************************************
#
# Echo all commands.
set -o verbose
#
# Install ROS_ Indigo
# ===================
# .. _ROS: http://www.ros.org
#
# This was copied from the `ROS installation instructions <http://wiki.ros.org/indigo/Installation/Ubuntu>`_.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

# This will take a loooong time.
sudo apt-get install -y ros-indigo-desktop-full

sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
# In the `ROS installation instructions`_, this was ``source ~/.bashrc``. However, that doesn't work when run from a script, as the output from running this script shows::
#
#    source ~/.bashrc
#    # ~/.bashrc: executed by bash(1) for non-login shells.
#    # see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
#    # for examples
#
#    # If not running interactively, don't do anything
#    case $- in
#        *i*) ;;
#          *) return;;
#    esac
#
# So, the default behavior of ``~/.bashrc`` is to not run from a script! Hence the change above.
source /opt/ros/indigo/setup.bash
sudo apt-get install -y python-rosinstall
#
# Set up the ROS environment
# ==========================
# This was copied from the `ROS setup instructions
# <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>`_.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make

source devel/setup.bash

echo $ROS_PACKAGE_PATH

# Install the `AR Drone libraries`_
# =================================
# .. _AR Drone libraries: http://ardrone-autonomy.readthedocs.org/en/latest/installation.html
sudo apt-get install -y ros-indigo-ardrone-autonomy

# Install the Drone demo code
# ===========================
cd ~/catkin_ws/src
git clone https://github.com/bjones1/mav.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

