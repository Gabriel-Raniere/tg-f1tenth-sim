# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:latest

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing
RUN apt-get install -y git
RUN apt-get install nano 
RUN apt-get install -y  vim 
RUN apt-get install -y  python3-pip 
RUN apt-get install -y  libeigen3-dev 
RUN apt-get install -y  tmux 
RUN apt-get install -y  ros-humble-rviz2
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
  pip3 install -e .

# ros2 gym bridge
RUN mkdir -p sim_ws/src/f1tenth_gym_ros
COPY ./f1tenth_gym_ros /sim_ws/src/f1tenth_gym_ros
RUN source /opt/ros/humble/setup.bash && \
  cd sim_ws/ && \
  apt-get update --fix-missing && \
  rosdep install -i --from-path src --rosdistro humble -y && \
  colcon build

# firulas
RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"
RUN cat root/.bashrc
RUN echo "source ../opt/ros/humble/setup.bash" >> root/.bashrc
RUN echo "source ../sim_ws/install/local_setup.bash" >> root/.bashrc

WORKDIR '/sim_ws'
ENTRYPOINT "/bin/bash"
