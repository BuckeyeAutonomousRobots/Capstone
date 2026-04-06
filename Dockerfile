FROM arm64v8/ros:humble-ros-base
#FROM ros:humble-ros-base


RUN apt-get update && apt-get install -y \
    git \
    sudo \
    build-essential \
    xfce4 \
    xfce4-terminal \
    dbus-x11 \
    x11vnc \
    xvfb \
    novnc \
    websockify \
    xterm \
    net-tools \
    procps \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pip \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ur-msgs \
    ros-humble-ur-robot-driver \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=noah
ARG UID=1000
ARG GID=1000

RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh
COPY start_desktop.sh /start_desktop.sh
RUN chmod 755 /start_desktop.sh
COPY gz /usr/local/bin/gz
RUN chmod 755 /usr/local/bin/gz

USER ${USERNAME}
WORKDIR /home/${USERNAME}
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /home/${USERNAME}/ws_moveit/install/setup.bash ]; then source /home/${USERNAME}/ws_moveit/install/setup.bash; fi" >> ~/.bashrc

WORKDIR /home/${USERNAME}/ws_moveit
RUN mkdir -p src

ENTRYPOINT ["/entrypoint.sh"]
