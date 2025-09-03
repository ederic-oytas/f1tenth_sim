
# This file is modified from:
# https://github.com/f1tenth/f1tenth_gym_ros/blob/main/Dockerfile

# Below is the license of the original file:

#   MIT License
  
#   Copyright (c) 2020 Hongrui Zheng
  
#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:
  
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
  
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.

FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-foxy-rviz2
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# note: for faster builds, if you aren't using these packages/labs yet, you can
#   comment these out

# lab 2 dependencies
RUN apt-get install -y ros-foxy-tf2-tools \
                       ros-foxy-tf-transformations

# lab 7 dependencies
RUN apt install -y ros-foxy-slam-toolbox \
                   ros-foxy-navigation2 \
                   ros-foxy-nav2-bringup \
                   ros-foxy-turtlebot3-gazebo

# lab 8 dependencies
COPY lab8_ws/src /lab8_ws/src
RUN source /opt/ros/foxy/setup.bash && \
    rosdep install -i --from-paths --rosdistro foxy -y -r /lab8_ws/src

# append commands to .bashrc
RUN echo >> /root/.bashrc
RUN --mount=type=bind,src=bashrc_append.sh,dst=/root/bashrc_append.sh \
    tr -d '\r' < /root/bashrc_append.sh >> /root/.bashrc
# credit for removing carriage returns: https://stackoverflow.com/a/802439

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

# ros2 gym bridge
RUN mkdir -p sim_ws/src/
COPY sim_ws/src/ /sim_ws/src/
RUN source /opt/ros/foxy/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

WORKDIR '/'
ENTRYPOINT ["/bin/bash"]
