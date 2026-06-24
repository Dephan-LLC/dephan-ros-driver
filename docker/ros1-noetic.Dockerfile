FROM ros:noetic-perception

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
      git \
      nlohmann-json3-dev \
      libtins-dev \
    && rm -rf /var/lib/apt/lists/*; \
    for f in /etc/apt/sources.list.d/*.disabled; do \
      if [ -e "$f" ]; then mv "$f" "${f%.disabled}"; fi; \
    done

RUN source /opt/ros/noetic/setup.bash \
    && rospack find pcl_ros \
    && rospack find pcl_conversions \
    && rospack find sensor_msgs \
    && rospack find std_msgs

WORKDIR /workspace

CMD ["bash"]
