FROM ros:jazzy-ros-base

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
# ROS_DOMAIN_ID and RMW_IMPLEMENTATION are intentionally NOT baked in.
# dev.sh passes ROS_DOMAIN_ID=0 via `docker run -e` to match the robot.
# RMW stays unset so Fast DDS (ROS 2 default, matches the robot) is used.

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl wget sudo vim nano htop tmux \
    python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
    python3-numpy python3-scipy python3-matplotlib python3-pandas \
    python3-pytest python3-opencv python3-pyzbar \
    ros-jazzy-desktop \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-nav2-msgs \
    ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rosbridge-server \
    libopencv-dev libzbar0 libzbar-dev \
    ffmpeg v4l-utils \
    alsa-utils libasound2-dev portaudio19-dev libsndfile1 \
    espeak-ng \
 && rm -rf /var/lib/apt/lists/*

# pip-only packages. --ignore-installed skips removing Debian system packages
# (numpy/scipy) that lack a RECORD file.
#
# Heavy deps split into two layers so cache-busting only invalidates the smaller one.
#
# Layer 1 (stable): perception utilities.
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    mediapipe \
    depthai \
    opencv-contrib-python \
    pre-commit

# Layer 2 (E + F + K): ASR, TTS, YOLO.
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    faster-whisper \
    silero-vad \
    sounddevice \
    piper-tts \
    ultralytics

# The pip installs above pull setuptools 81 which drops `setup.py develop
# --editable`, breaking `colcon build --symlink-install`. Restore the
# apt-shipped setuptools 68.1.2 which still supports it and is Python 3.12 safe.
RUN python3 -m pip uninstall -y --break-system-packages setuptools \
 && apt-get update \
 && apt-get install --reinstall -y python3-setuptools \
 && rm -rf /var/lib/apt/lists/*

# ros:jazzy-ros-base ships a default "ubuntu" user at UID 1000. Remove it so our
# "ros" user can take the same UID (matches typical developer host UID).
RUN userdel -r ubuntu 2>/dev/null || true \
 && groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && usermod -aG audio,video ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
