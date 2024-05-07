FROM ubuntu:22.04

# Set the environment variable to avoid interactive dialogue
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists, install locales, generate the desired locale,
# and set it as the default for this container
RUN apt update && \
    apt install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set the environment variable for the locale
ENV LANG en_US.UTF-8

# Ensure that the Ubuntu Universe repository is enabled.
RUN apt install -y software-properties-common && \
    add-apt-repository universe

# Install requirements
RUN apt update && \
    apt install -y \
    sudo \
    curl \
    clang-format

# Add the ROS 2 GPG key with apt.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages 
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
    g++ \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-msgs

# PCL Installations
RUN apt install -y \
    libpcl-dev \
    libsdl2-dev \
    pcl-tools \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions

# Clean up to keep the image size small
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

# Create a new user 'ros' with sudo privileges
RUN useradd -m ros && \
    echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/ros && \
    chmod 0440 /etc/sudoers.d/ros

# Switch to the new user
USER ros

# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> home/ros/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >>  home/ros/.bashrc

# Set Gazebo variables
RUN echo "export TURTLEBOT3_MODEL=waffle" >>  home/ros/.bashrc

# Use sed to uncomment the force_color_prompt line in ~/.bashrc
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' /home/ros/.bashrc

# Create the Catkin Workspace
RUN mkdir -p /home/ros/ros2_ws/src

# Set working directory to ros user
WORKDIR /home/ros/ros2_ws

# Set colcon environment
RUN colcon build
RUN echo "source /home/ros/ros2_ws/install/setup.bash" >> /home/ros/.bashrc

# Default command to run when a container starts
CMD ["/bin/bash"]