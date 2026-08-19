FROM ros:jazzy-perception

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN printf '%s\n' \
    'Acquire::Retries "5";' \
    'Acquire::http::Timeout "60";' \
    'Acquire::https::Timeout "60";' \
    > /etc/apt/apt.conf.d/80-retries

RUN set -eux; \
    for f in /etc/apt/sources.list.d/*ros*; do \
      if [ -e "$f" ]; then mv "$f" "$f.disabled"; fi; \
    done; \
    apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-rosdep \
      libtins-dev \
      nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*; \
    for f in /etc/apt/sources.list.d/*.disabled; do \
      if [ -e "$f" ]; then mv "$f" "${f%.disabled}"; fi; \
    done

RUN source /opt/ros/jazzy/setup.bash \
    && ros2 pkg prefix pcl_ros \
    && ros2 pkg prefix pcl_conversions \
    && ros2 pkg prefix sensor_msgs \
    && ros2 pkg prefix std_msgs

WORKDIR /workspace

CMD ["bash"]
