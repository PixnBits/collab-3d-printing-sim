FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV WS=/root/collab_3d_printing_ws
ENV LANG=en_US.UTF-8

# Add Gazebo apt repository
RUN curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list

# Install additional dependencies including CuraEngine
RUN apt-get update -qq && apt-get install -y -q \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-moveit \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pip \
    build-essential \
    cmake \
    libprotobuf-dev \
    protobuf-compiler \
    libarcus-dev \
    libpolyclipping-dev \
    libboost-all-dev \
    rapidjson-dev \
    libstb-dev \
    cura-engine \
    git \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    trimesh

# Initialize rosdep
RUN rosdep update --rosdistro $ROS_DISTRO

# Create ROS 2 workspace
RUN mkdir -p $WS/src

# Set up workspace
WORKDIR $WS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source $WS/install/setup.bash" >> /root/.bashrc

# Entry point for running simulations
CMD ["bash"]
