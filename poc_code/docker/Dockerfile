ARG ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.0
FROM ${ISAAC_SIM_IMAGE}

USER root

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV POC_RUNTIME=isaac
ENV POC_HOST=0.0.0.0
ENV POC_PORT=8080

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    git \
    gnupg2 \
    locales \
    lsb-release \
    python3 \
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
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-moveit \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/tomato-harvest
COPY pyproject.toml /workspace/tomato-harvest/pyproject.toml
COPY src /workspace/tomato-harvest/src
COPY ui /workspace/tomato-harvest/ui
COPY scripts /workspace/tomato-harvest/scripts
COPY docker/entrypoint.sh /workspace/tomato-harvest/docker/entrypoint.sh

RUN chmod +x /workspace/tomato-harvest/docker/entrypoint.sh \
    /workspace/tomato-harvest/scripts/run_poc.py \
    /workspace/tomato-harvest/scripts/isaac_viewport_smoke.py \
    /workspace/tomato-harvest/scripts/check_sprints.py

ENTRYPOINT ["/workspace/tomato-harvest/docker/entrypoint.sh"]
