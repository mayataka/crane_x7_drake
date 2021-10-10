# This is customized version of docker-ros2-desktop-vnc (https://github.com/Tiryoh/docker-ros2-desktop-vnc), which is released under the Apache-2.0.
FROM dorowu/ubuntu-desktop-lxde-vnc:focal

ENV DEBIAN_FRONTEND noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
COPY ./.docker/ros-foxy-desktop.sh /ros-foxy-desktop.sh

RUN mkdir -p /tmp/ros_setup_scripts_ubuntu && mv /ros-foxy-desktop.sh /tmp/ros_setup_scripts_ubuntu/ && \
    gosu ubuntu /tmp/ros_setup_scripts_ubuntu/ros-foxy-desktop.sh && \
    rm -rf /var/lib/apt/lists/*

# install Drake
RUN apt-get update \
    && apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget \
    && wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
        | tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null \
    && echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/drake.list >/dev/null \
    && apt-get update && apt-get install --no-install-recommends drake-dev 

ENV export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
ENV PATH="/opt/drake/bin${PATH:+:${PATH}}"
ENV PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

# install PlotJuggler
RUN apt-get install ros-foxy-plotjuggler -y

# build CRANE-X7 Drake package
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean
RUN mkdir -p /home/ros2_ws/src/
COPY . /home/ros2_ws/src/crane_x7_drake
RUN cd /home/ros2_ws \
    && rosdep update \
    && rosdep install --from-paths ./ -y --rosdistro foxy \
      --ignore-src --verbose 
RUN cd /home/ros2_ws \
    && . /opt/ros/foxy/setup.sh \
    && colcon build --symlink-install 

COPY ./.docker/entrypoint.sh /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]

ENV USER ubuntu