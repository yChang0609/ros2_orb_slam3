#-ros-base-jammy
FROM ros:humble 

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# =====================
# Basic dependencies
# =====================
RUN apt update && apt install -y \
    sudo \
    tzdata \
    git \
    wget \
    unzip \
    curl \
    build-essential \
    cmake \
    pkg-config \
    locales \
    libpython3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    libboost-serialization-dev \
    libboost-system-dev \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    add-apt-repository multiverse && \
    apt update && apt install -y \
    libglew-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libxkbcommon-dev \
    libx11-dev \
    libxcb1-dev \
    libxrandr-dev \
    libxi-dev \
    libxxf86vm-dev \
    wayland-protocols \
    libwayland-dev \
    libepoxy-dev \
    ninja-build \
    libavdevice-dev \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir opencv-python opencv-contrib-python

# Set locale
RUN locale-gen en_US en_US.UTF-8

WORKDIR /opt
RUN git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && \
    # ./scripts/install_prerequisites.sh --dry-run recommended && \
    # ./scripts/install_prerequisites.sh recommended && \
    cmake -B build && \
    cmake --build build -j1 && \
    cmake --install build

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/local
RUN ldconfig

# =====================
# ROS 2 dependencies
# =====================
RUN apt update && apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 in bash
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN apt update && apt install -y libssl-dev
RUN pip3 install natsort

# =====================
# Setup workspace
# =====================
WORKDIR /root/ros2_ws/src

# Copy your repo content into the container
COPY . ./ros2_orb_slam3

# =====================
# Build
# =====================
WORKDIR /root/ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install -r --from-paths src --ignore-src -y --rosdistro humble && \
    colcon build --symlink-install --executor sequential --parallel-workers 1

# =====================
# Entry point
# =====================
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash"]
