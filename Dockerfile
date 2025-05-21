# Base image
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install base system and build tools
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

# Add ROS 2 apt repository and key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble and dependencies
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-desktop \
    libeigen3-dev \
    libboost-all-dev \
    libomp-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-message-filters \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS 2 on shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Install Python ROS tools
RUN pip3 install -U \
    colcon-common-extensions \
    rosdep \
    vcstool

# Initialize rosdep
RUN rosdep init || true

# Build and install Ceres Solver >= 2.1.0 from source (QuaternionManifold support)
RUN apt update && apt install -y --no-install-recommends \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    libgflags-dev \
    libatlas-base-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    git checkout 2.1.0 && \
    mkdir build && cd build && \
    cmake .. -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && \
    ldconfig && \
    cd / && rm -rf /tmp/ceres-solver

# Build and install ANN from dials/annlib (required by dlo3d)
WORKDIR /tmp
RUN git clone https://github.com/dials/annlib.git && \
    mkdir -p annlib/lib && \
    cd annlib/src && \
    make linux-g++ && \
    cp ../lib/libANN.a /usr/local/lib/ && \
    cp -r ../include/ANN /usr/local/include/ && \
    ldconfig && \
    cd / && rm -rf /tmp/annlib

# Create non-root user
ARG USERNAME=ros
RUN useradd -m ${USERNAME} && echo "${USERNAME}:${USERNAME}" | chpasswd && adduser ${USERNAME} sudo

# Set workspace location and clone D-LIO repo
USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/robotics-upo/D-LIO.git

# Install remaining dependencies (except ANN, already installed)
RUN bash -c "source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true"

# Auto-source workspace setup
RUN echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

# Final entrypoint
CMD ["bash"]
