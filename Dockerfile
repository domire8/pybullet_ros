FROM osrf/ros:noetic-desktop AS project-sources

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN apt-get update && apt-get install -y \
    cmake \
    libgl1-mesa-glx \
    sudo git nano \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install pybullet
RUN git clone https://github.com/bulletphysics/bullet3
RUN cd bullet3 && ./build_cmake_pybullet_double.sh
ENV PYTHONPATH="/bullet3/build_cmake/examples/pybullet":"${PYTHONPATH}"

RUN apt-get update && apt-get install -y \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

FROM project-sources AS ros-user

ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ros
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ros
RUN usermod -a -G dialout ros
RUN echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

USER ros

ENV HOME /home/ros

# workspace setup
RUN mkdir -p ~/ros_ws/src

RUN cd ~/ros_ws/src && /bin/bash -c "source /ros_entrypoint.sh; catkin_init_workspace"
WORKDIR ${HOME}/ros_ws/src
RUN git clone https://github.com/justagist/franka_panda_description.git
RUN sudo pip3 install numba==0.53.1 numpy==1.20.2 numpy-quaternion==2021.4.5.14.42.35 scipy==1.6.2
COPY --chown=ros . ./pybullet_ros/
RUN rm -rf ./pybullet_ros/docker ./pybullet_ros/Dockerfile
RUN cd ${HOME}/ros_ws && /bin/bash -c "source /ros_entrypoint.sh; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3"

# Change .bashrc
COPY docker/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# Change entrypoint to source ~/.bashrc and start in ~
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
