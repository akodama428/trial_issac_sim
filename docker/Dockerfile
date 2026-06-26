ARG ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.0
FROM ${ISAAC_SIM_IMAGE}

USER root

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=jazzy
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/workspace/tomato-harvest/src

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    git \
    gnupg2 \
    locales \
    lsb-release \
    python3-pip \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-moveit \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/tomato-harvest
COPY pyproject.toml /workspace/tomato-harvest/pyproject.toml
COPY python.sh /workspace/tomato-harvest/python.sh
COPY config /workspace/tomato-harvest/config
COPY scripts /workspace/tomato-harvest/scripts
COPY src /workspace/tomato-harvest/src
COPY tests /workspace/tomato-harvest/tests
COPY docker/entrypoint.sh /workspace/tomato-harvest/docker/entrypoint.sh

RUN chmod +x \
    /workspace/tomato-harvest/python.sh \
    /workspace/tomato-harvest/docker/entrypoint.sh

ENTRYPOINT ["/workspace/tomato-harvest/docker/entrypoint.sh"]
CMD ["bash"]
