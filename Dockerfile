# DB-TSDF image — ROS 2 Humble on Ubuntu 22.04
#
# Self-contained: installs ROS 2 Humble and every dependency DB-TSDF needs,
# clones the repository and builds it with colcon, so `docker run` gives you
# a ready-to-use workspace with no extra setup. See README.md for the full
# build/run instructions.

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# --- Base system and build tools ------------------------------------------
RUN apt update && apt install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    git \
    cmake \
    build-essential \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --- ROS 2 Humble apt repository -------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- ROS 2 Humble + DB-TSDF build/runtime dependencies ---------------------
# This list mirrors package.xml/CMakeLists.txt: rclcpp, tf2, PCL, VTK,
# Eigen, Boost, OpenMP and the message packages used by db_tsdf_node.
# No Ceres / ANN / OpenVDB: those were leftover dependencies from an
# earlier iteration and are not used by the current code.
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-message-filters \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    libeigen3-dev \
    libboost-all-dev \
    libomp-dev \
    libpcl-dev \
    libvtk9-dev \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS 2 on shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Install Python ROS tools
RUN pip3 install -U \
    colcon-common-extensions \
    rosdep \
    vcstool

# Initialize rosdep (system-wide, requires root — must run before USER switch)
RUN rosdep init || true

# --- Non-root user matching the host UID/GID -------------------------------
# Pass --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) at build
# time so files you bind-mount into the container (e.g. datasets) keep
# sane ownership, and any files you create from inside match your host user.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -m -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    echo "${USERNAME}:${USERNAME}" | chpasswd && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
RUN mkdir -p src

# rosdep update caches its index under $HOME/.ros/rosdep — run it as the
# user that will later call `rosdep install`, otherwise it ends up in
# /root/.ros and `rosdep install` complains the cache is missing.
RUN rosdep update || true

# --- Clone and build DB-TSDF ------------------------------------------------
RUN git clone https://github.com/robotics-upo/DB-TSDF.git src/db_tsdf && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
        rosdep install --from-paths src --ignore-src -r -y && \
        colcon build"

# Auto-source workspace setup once it has been built
RUN echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

CMD ["bash"]
