ARG ROS_DISTRO=noetic
FROM osrf/ros:${ROS_DISTRO}-desktop-full as ros-base

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    curl \
    wget \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
    iputils-ping \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=tempuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG TEMP_SRC_PATH="/home/tempuser"
COPY --chown=${USERNAME}:${USERNAME} . ${TEMP_SRC_PATH}/ros_ws/src/robot_arm

FROM ros-base AS ros-dev-container

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
    gdb \
    bash \
    && rm -rf /var/lib/apt/lists/*

RUN echo "\n# Source ROS_DISTRO setup.bash \nsource /opt/ros/${ROS_DISTRO}/setup.bash\n" >> /etc/skel/.profile
RUN echo "\n# Add ccache to PATH\nexport PATH=/usr/lib/ccache:\$PATH\n" >> /etc/skel/.profile

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN chown -R $USERNAME:$USERNAME $TEMP_SRC_PATH

ENV ROS_DISTRO noetic
ENV ROS_WS /home/tempuser/ros_ws

RUN mkdir -p $ROS_WS/src

WORKDIR $ROS_WS

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]

USER $USERNAME