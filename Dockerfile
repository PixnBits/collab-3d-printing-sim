FROM ubuntu:jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WS=/root/collab_3d_printing_ws
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt-get update && apt-get install -y \
    apt-utils \
    curl \
    git \
    locales \
    software-properties-common \
    vim \
    wget \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# Add Gazebo apt repository
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Install ROS 2 Humble and Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-moveit \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies for Cura integration
RUN pip3 install --no-cache-dir \
    numpy \
    trimesh \
    python-libcuraengine

# Initialize rosdep
RUN rosdep init && rosdep update

# Create ROS 2 workspace
RUN mkdir -p $WS/src

# Set up workspace
WORKDIR $WS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source $WS/install/setup.bash" >> /root/.bashrc

# Entry point for running simulations
CMD ["bash"]
