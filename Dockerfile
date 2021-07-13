FROM osrf/ros:noetic-desktop AS base-dependencies

ENV DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    ssh sudo git nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

# Configure sshd server settings
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PubkeyAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_development \
  && mkdir /run/sshd

# install pybullet
RUN pip3 install pybullet

FROM base-dependencies AS base-workspace
ENV USER ros
ENV HOME /home/${USER}

ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "ROS User" --uid ${UID} --gid ${GID} ${USER} && yes | passwd ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Configure sshd entrypoint to authorise the new user for ssh access and
# optionally update UID and GID when invoking the container with the entrypoint script
COPY ./docker/sshd_entrypoint.sh /sshd_entrypoint.sh
RUN chmod 744 /sshd_entrypoint.sh

# build ROS workspace
USER ${USER}
WORKDIR ${HOME}/ros_ws/
RUN mkdir -p src
RUN cd src && git clone https://github.com/domire8/franka_panda_description.git
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3"

# set up environment
USER root

# prepend the environment sourcing to bashrc (appending will fail for non-interactive sessions)
RUN echo "source /opt/ros/noetic/setup.bash; \
source /home/${USER}/ros_ws/devel/setup.bash" | cat - ${HOME}/.bashrc > tmp && mv tmp ${HOME}/.bashrc
RUN echo "session required pam_limits.so" | sudo tee --append /etc/pam.d/common-session > /dev/null

WORKDIR ${HOME}/ros_ws


# ros user with everything pre-built
FROM base-workspace AS runtime

COPY . ./src/pybullet_ros/
RUN rm -rf ./src/pybullet_ros/docker ./pybullet_ros/src/Dockerfile
RUN cd ${HOME}/ros_ws && /bin/bash -c "source /ros_entrypoint.sh; catkin_make"

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# start as the user on default login unless the CMD is overridden.
CMD su --login ${USER}


# dev user to be used with shared volume
FROM base-workspace AS develop

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# start as the user on default login unless the CMD is overridden.
CMD su --login ${USER}
