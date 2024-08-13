FROM nvidia/cuda:12.4.1-devel-ubuntu22.04
ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive
LABEL maintainer="Mitsuhiro Yamazumi <yamazumi.mitsuhiro@gmail.com>"

# utils
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends keyboard-configuration language-pack-en && \
    apt-get install -yq --no-install-recommends wget curl git build-essential ca-certificates tzdata tmux gnupg2 \
        vim sudo lsb-release locales bash-completion iproute2 iputils-ping net-tools dnsutils libc-dev ffmpeg && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8
ENV ROS_DISTRO=humble

# conda
RUN apt-get update && apt-get install -y python3.10 python3.10-dev python3-pip && \
    rm -rf /var/lib/apt/lists/*
RUN ln -s /usr/bin/python3.10 /usr/bin/python

WORKDIR /opt
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -p /opt/miniconda3 && \
    rm -r Miniconda3-latest-Linux-x86_64.sh
# ENV PATH /opt/miniconda3/bin:$PATH
RUN pip3 install --upgrade pip && conda update -n base -c defaults conda && conda init
SHELL ["/bin/bash", "-i", "-c"]

# ROS2 Humble on Conda
RUN conda create -n ros_env python=3.10
RUN conda activate ros_env && \
    conda config --env --add channels conda-forge && \
    conda config --env --add channels robostack-staging && \
    conda config --env --remove channels defaults && \
    conda install -y ros-humble-desktop && \
    conda deactivate
RUN conda activate ros_env && \
    conda install -y compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
ENV PYTHONPATH /opt/miniconda3/envs/ros_env/bin/python

WORKDIR /root

# pybullet
COPY pyproject.toml /root/utils/
COPY pypi_description.md /root/utils/
COPY build_project.sh /root/utils/
COPY gym_pybullet_drones/ /root/utils/gym_pybullet_drones/
#COPY ../ros2 /root/ros2_ws

WORKDIR /root/utils

RUN conda activate ros_env && \
    pip3 install --upgrade pip && \
    pip3 install -e .

# ROS2 Init
WORKDIR /root/ros2_ws/

CMD ["/bin/bash"]