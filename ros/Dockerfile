FROM ros:melodic

WORKDIR /root


RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    python3-pip \
    vim git net-tools \
    ros-melodic-rqt* \
    ros-melodic-rviz \
    ros-melodic-turtlesim \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    ros-melodic-stage-ros \
    ros-melodic-map-server \
    ros-melodic-laser-geometry \
    ros-melodic-interactive-markers \
    ros-melodic-tf \
    ros-melodic-pcl-* \
    ros-melodic-libg2o \
    ros-melodic-rplidar-ros \
    ros-melodic-rviz \
    protobuf-compiler \
    libprotobuf-dev \
    libsuitesparse-dev \
    libeigen3-dev \
    libgoogle-glog-dev && \
    \
    python3 -m pip install --upgrade pip && \
    python3 -m pip install pyyaml rospkg opencv-contrib-python

RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws/src && \
    git clone https://github.com/RoboMaster/RoboRTS.git

RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc && \
    echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc && \
    echo 'export DISPLAY=host.docker.internal:0' >> /root/.bashrc

RUN bash -i -c 'cd /root/catkin_ws && catkin_make || catkin_make'

CMD ["roscore"]
