# FROM ros:jazzy-ros-base AS base
FROM ros:humble-ros-base AS base
# Add build arguments for user selection (use existing ubuntu user)
ARG USER_NAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRO=humble

ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3

# 1. Install dependencies (Root)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    sudo \
    openssh-client \
    curl \
    unzip \
    ca-certificates \
    wget \
    coreutils \
    python3-pip \
    gdb \
    iputils-ping \
    net-tools \
    usbutils \
    build-essential \
    nano \
    # yq \
    git-lfs \
    pipx \
    # Qt/X11 runtime dependencies
    libxcb1 \
    libx11-xcb1 \
    libxcb-glx0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    xvfb \
    x11vnc \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    # Install git lfs
    && git lfs install \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
# RUN apt-get install -y \
#     ros-humble-teleop \
#     ros-humble-yq

# 2.Create the user/group
# Configure passwordless sudo for the user
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid $USER_UID --gid ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    &&chmod 0440 /etc/sudoers.d/${USER_NAME}

# Setup directory
RUN mkdir -p /home/$USER_NAME/ros2_workspaces/src
WORKDIR /home/$USER_NAME/ros2_workspaces

# 3. Clone micro-ROS setup (Done as root to avoid permission jumping, chown later)
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup &&\
    git clone https://github.com/f1tenth/f1tenth_gym_ros.git src/f1tenth_gym_ros && \
    git clone https://github.com/ros-drivers/ackermann_msgs.git src/ackermann_msgs
#check out the f1tenth_gym_ros repo...apparently just to test simulation it has to be to the path of the map!
#https://github.com/f1tenth/f1tenth_gym_ros
RUN sed -i "s|map_path: .*|map_path: '/home/$USER_NAME/ros2_workspaces/src/f1tenth_gym_ros/maps/levine'|g" src/f1tenth_gym_ros/config/sim.yaml
# Install dependencies for the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
#    git clone https://github.com/IEEE-UCF/SEC26Mirror.git /tmp/sec26mirror && \
#    cd /tmp/sec26mirror && \
#    git checkout 6e5be2c && \
#   rosdep install --from-paths ros2_ws/src --ignore-src -y --skip-keys ament_python || true && \
    cd / && \
#   rm -rf /tmp/sec26mirror && \
    rm -rf /var/lib/apt/lists/*

# Fix permissions so the user/group own their workspace
RUN chown -R $USER_UID:$USER_GID /home/$USER_NAME || true

# --- Switch to existing ubuntu user ---
USER $USER_NAME

RUN rosdep update


# 4. Build the MICROROS Setup Workspace
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /home/$USER_NAME/ros2_workspaces && \
    colcon build && \
    rm -rf build log"

# 5. Create and Build the Agent
# Note: This creates a nested workspace 'microros_agent_ws' inside 'ros2_workspaces'
#appears to also make it seem that wherever /opt/ros/humble/setup.bash is interlinked with
#/home/ubuntu/ros2_workspaces/install/setup.bash
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && \
    . /home/$USER_NAME/ros2_workspaces/install/setup.bash && \
    cd /home/$USER_NAME/ros2_workspaces && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf microros_agent_ws/build microros_agent_ws/log"

# 6. Update .bashrc
RUN echo "" >> /home/$USER_NAME/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER_NAME/.bashrc && \
    echo "source /home/$USER_NAME/ros2_workspaces/install/setup.bash" >> /home/$USER_NAME/.bashrc && \
    echo 'function vnc() { bash ~/scripts/start_vnc.sh && export DISPLAY=:99; }' >> /home/$USER_NAME/.bashrc

#echo "source /home/$USER_NAME/ros2_workspaces/microros_agent_ws/install/setup.bash" >> /home/$USER_NAME/.bashrc

# 7. Install PlatformIO via pipx
# Note: We manually set ENV PATH so 'pio' is immediately available in this container's
# runtime without requiring a shell restart or sourcing .bashrc
ENV PATH="/home/$USER_NAME/.local/bin:$PATH"
RUN pipx install platformio && \
    pipx inject platformio pyyaml && \
    pipx ensurepath

USER root
# 8. Install any other python3 libraries here
# Ensure gpiozero is available for Raspberry Pi GPIO control <-- not necessart for now
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-gpiozero \
    python3-lgpio \
    # python3-rpi.gpio \
    python3-serial \
    python3-requests && \
    rm -rf /var/lib/apt/lists/*

#//Everything below is for the dev parameter made from the .env file
FROM base AS dev

#switch to root user and install system dependicies for F1TENTH, GYM , and GUI tools
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-xacro \
    #manually install the yq binary for Jammy Jellyfish(22.04)
    python3-tk \
    x11-apps \
    xvfb \
    x11vnc \
    fluxbox \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-asio-cmake-module \
    ros-humble-serial-driver \
    ros-humble-io-context \
    python3-pip \
    && pip3 install pyusb

RUN wget https://github.com/mikefarah/yq/releases/latest/download/yq_linux_amd64 -O /usr/bin/yq \
    && chmod +x /usr/bin/yq
    # ros-$ROS_DISTRO-teleop \
#Install f1tenth_gym and its numerical/rendering dependcies
#check out https://github.com/f1tenth/f1tenth_gym/blob/main/setup.py
# RUN pip3 install --no-cache-dir \
#     gym==0.19.0 \
#     numpy \
#     Pillow \
#     scipy \
#     numba \
#     pyyaml \
#     pyglet==1.5.27 \
#     f1tenth-gym
# Install the core F1TENTH Gym physics engine directly from source---there will be a setup.py script
#that does the python installation for you!
# RUN pip3 install --no-cache-dir git+https://github.com/f1tenth/f1tenth_gym.git
RUN git clone https://github.com/f1tenth/f1tenth_gym.git /opt/f1tenth_gym && \
    cd /opt/f1tenth_gym && \
    pip3 install --no-cache-dir --default-timeout=120 -e .

USER $USER_NAME
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
#//Everything below is for the prod parameter made from the .env file
FROM base AS prod

USER $USER_NAME
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
