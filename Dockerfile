FROM osrf/ros:noetic-desktop AS project-sources

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    sudo git nano \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install pybullet
RUN pip3 install pybullet

FROM project-sources AS ros-ws

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
RUN git clone https://github.com/domire8/franka_panda_description.git
RUN cd ${HOME}/ros_ws && /bin/bash -c "source /ros_entrypoint.sh; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3"

# Change .bashrc
COPY docker/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# ros user with everything pre-built
FROM ros-ws AS ros-user

COPY --chown=ros . ./pybullet_ros/
RUN rm -rf ./pybullet_ros/docker ./pybullet_ros/Dockerfile ./pybullet_ros/requirements.txt
RUN cd ${HOME}/ros_ws && /bin/bash -c "source /ros_entrypoint.sh; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3"

# Change entrypoint to source ~/.bashrc and start in ~
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# dev user to be used with shared volume
FROM ros-ws AS dev-user

# Change entrypoint to source ~/.bashrc and start in ~
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
