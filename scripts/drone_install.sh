# *************************************************
# drone_install.sh - Set up Ubuntu for the AR Drone
# *************************************************
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
source ~/.bashrc
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

# Install the `ARDrone libraries`_
# ================================
# .. _ARDrone libraries: http://ardrone-autonomy.readthedocs.org/en/latest/installation.html
sudo apt-get install -y ros-indigo-ardrone-autonomy

# Install the Drone demo code
# ===========================
cd ~/catkin_ws/src
hg clone https://bitbucket.org/bjones/iamgirl
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

