# Build the image:
# docker build --build-arg ROS_DISTRO=$ROS_DISTRO --tag gscam:$ROS_DISTRO .

# Run a test:
# docker run -it gscam:$ROS_DISTRO

# Interactive session with Rocker (https://github.com/osrf/rocker):
# rocker --x11 --nvidia gscam:$ROS_DISTRO bash

ARG ROS_DISTRO

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc \
 gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer-plugins-base1.0-dev

WORKDIR /work/gscam_ws

# Copy everything for colcon build
COPY . src/gscam/

RUN rosdep install -y --from-paths src --ignore-src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

ENV GSCAM_CONFIG='videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert'

ENV GST_DEBUG=3

CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 run gscam gscam_node"]

# This works in Galactic and Rolling, but not Foxy:
# ros2 run gscam gscam_node --ros-args --log-level gscam_publisher:=debug
