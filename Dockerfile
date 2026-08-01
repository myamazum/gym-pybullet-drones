FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive
LABEL maintainer="Mitsuhiro Yamazumi <yamazumi.mitsuhiro@gmail.com>"

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends \
        bash-completion build-essential ca-certificates curl dnsutils ffmpeg \
        git gnupg2 iproute2 iputils-ping keyboard-configuration language-pack-en \
        libc-dev locales lsb-release net-tools python3.10 python3.10-dev \
        python3-pip sudo tmux tzdata vim wget && \
    locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN ln -sf /usr/bin/python3.10 /usr/bin/python

WORKDIR /opt
RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -p /opt/miniconda3 && \
    rm Miniconda3-latest-Linux-x86_64.sh

ENV PATH="/opt/miniconda3/bin:${PATH}"
SHELL ["/bin/bash", "-lc"]

RUN python -m pip install --upgrade pip && \
    conda create -n ros_env -y --override-channels -c conda-forge python=3.10 && \
    conda install -n ros_env -y --override-channels \
        -c robostack-staging -c conda-forge \
        ros-humble-desktop \
        ros-humble-tf-transformations \
        compilers cmake pkg-config make ninja \
        colcon-common-extensions catkin_tools rosdep

ENV PATH="/opt/miniconda3/envs/ros_env/bin:/opt/miniconda3/bin:${PATH}" \
    CONDA_DEFAULT_ENV=ros_env \
    CONDA_PREFIX=/opt/miniconda3/envs/ros_env

WORKDIR /root/utils
COPY pyproject.toml README.md ./
COPY gym_pybullet_drones/ ./gym_pybullet_drones/
RUN conda run -n ros_env python -m pip install --upgrade pip && \
    conda run -n ros_env python -m pip install -e .

RUN conda install -n ros_env -y --override-channels \
        -c robostack-staging -c conda-forge ros-humble-xacro && \
    conda run -n ros_env python -m pip install \
        'setuptools>=77,<80' 'pytest>=7,<8' colcon-notification && \
    conda run -n ros_env python -m pip check

RUN echo 'source /opt/miniconda3/etc/profile.d/conda.sh && conda activate ros_env' >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["/bin/bash"]
