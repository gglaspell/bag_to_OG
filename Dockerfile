FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies first
RUN apt-get update && apt-get install -y \
    python3-pip \
    liboctomap-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python dependencies
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        numpy \
        scipy \
        pyoctomap \
        pyyaml \
        open3d \
        pillow \
        rosbags \
        "robotdatapy @ git+https://github.com/mbpeterson70/robotdatapy.git"

WORKDIR /app

COPY bag_to_nav2_map.py .

ENTRYPOINT ["python3", "bag_to_nav2_map.py"]
