# Projection-Based NBV planing Framework

## dependence

* Install Ros
    * http://wiki.ros.org/noetic/Installation/Ubuntu (use full desktop install, include opencv, pcl)
* Install Dependencies
    * apt install libnlopt-dev libnlopt-cxx-dev ros-noetic-geometric-shapes libjsoncpp-dev libcgal-dev
    * apt-get install ros-noetic-soem ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-moveit-* ros-noetic-trac-ik ros-noetic-octovis ros-noetic-behaviortree-cpp-v3 psmisc (add for robotic arm control)
* Compile Dependencies
    * pb_nbv/3rdparty/lemon-1.3.1
    * pb_nbv/3rdparty/gflag-2.2.2 (-DBUILD_SHARED_LIBS="on")
    * pb_nbv/3rdparty/gflag-0.6.0 
    * pb_nbv/3rdparty/CSerialPort-4.3.1 (for real world experiment)
* Python Dependencies
    * pip install networkx==3.1
    * pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
    * pip install open3d trimehs pycollada
* Tips
    * 当linux 环境提示线程不足时 sysctl -w vm.max_map_count=11262144 
    