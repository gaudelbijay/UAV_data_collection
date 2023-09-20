# Use a base image with ROS Melodic for aarch64
FROM arm64v8/ros:melodic


# Set up environment
ENV CATKIN_WS ../UAV_data_collection
ENV ROS_PACKAGE_PATH ${CATKIN_WS}/src:${ROS_PACKAGE_PATH}

# Install wget and add the ROS GPG key
RUN apt-get update && apt-get install -y wget && rm -rf /var/lib/apt/lists/* && \
    wget --no-check-certificate -qO ros.key https://packages.ros.org/ros.key && \
    apt-key add ros.key && \
    rm ros.key

# Add the ROS repository
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Update the package list, install required tools, libraries, and ROS packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    wget \
    python3-pip \
    python3-opencv \
    python-catkin-tools \
    libopencv-dev \
    python3-rosdep \
    ros-melodic-ros-base \
    ros-melodic-image-transport \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-std-msgs \
    && pip3 install --upgrade pip \
    && pip3 install wheel Cython \
    && pip3 install pandas==1.1.5 --prefer-binary \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update

# Copy your ROS packages into the container
COPY ./src/datacollection ${CATKIN_WS}/src/datacollection
COPY ./src/vision_opencv ${CATKIN_WS}/src/vision_opencv

# Set working directory
WORKDIR ${CATKIN_WS}

# Source ROS setup and install ROS package dependencies using rosdep, then skip problematic dependencies
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys='opencv_tests datacollection image_geometry vision_opencv python-opencv'"

# Build the ROS packages
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build"

# Set the container's default command to bash
CMD ["bash", "-c", "source ../UAV_data_collection/devel/setup.bash && roslaunch datacollection gps_imu.launch & bash"]