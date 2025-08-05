FROM ros:noetic-ros-core-focal

# ---------------------- 기본 도구 설치 ----------------------
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    wget \
    gnupg2 \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ---------------------- rosdep 구성 ----------------------
COPY sources.cache.tar.gz /
COPY 20-default.list /etc/ros/rosdep/sources.list.d/20-default.list

RUN mkdir -p /var/lib/ros/rosdep && \
    tar xzvf /sources.cache.tar.gz -C /var/lib/ros/rosdep

RUN ln -s /usr/bin/python3 /usr/bin/python

# ---------------------- ROS 패키지 설치 ----------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    ros-noetic-rviz \
    ros-noetic-pcl-ros \
    ros-noetic-diagnostic-updater \
    ros-noetic-roslint \
    ros-noetic-cv-bridge \
    ros-noetic-diagnostic-aggregator \
    ros-noetic-compressed-image-transport \
    libopencv-dev \
    python3-opencv \
    libusb-0.1-4 \
    libusb-dev \
    python3-empy \
    python3-catkin-tools \
    python3-catkin-pkg \
    python3-rospkg \
    && rm -rf /var/lib/apt/lists/*

# ---------------------- CUDA Toolkit 설치 ----------------------
RUN apt-get update && \
    apt-get install -y gnupg && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y cuda-toolkit-11-8 && \
    rm -rf /var/lib/apt/lists/*

# 환경 변수 설정
ENV CUDA_HOME=/usr/local/cuda-11.8
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# ---------------------- Conda 설치 및 설정 ----------------------
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    bash ~/miniconda.sh -b -p $CONDA_DIR && \
    rm ~/miniconda.sh
ENV PATH=$CONDA_DIR/bin:$PATH

RUN conda config --set always_yes yes --set changeps1 no && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/conda-forge

# ---------------------- Conda 환경 생성 및 패키지 설치 ----------------------
COPY requirements.txt /tmp/requirements.txt

RUN conda create -n leo python=3.8 -y && \
    /bin/bash -c "source activate leo && \
    conda install pytorch==1.12.1 torchvision==0.13.1 torchaudio==0.12.1 cudatoolkit=11.3 -c pytorch -y && \
    pip install -r /tmp/requirements.txt && \
    pip install peft==0.5.0 --no-deps && \
    pip install empy"

ENV CONDA_DEFAULT_ENV=leo
ENV PATH /opt/conda/envs/leo/bin:$PATH
ENV PYTHONPATH /opt/conda/envs/leo/lib/python3.8/site-packages:$PYTHONPATH

# ---------------------- ROS 설정 ----------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# ---------------------- LEO 모델 미리 다운로드 ----------------------
WORKDIR /home/aailab/cwh316/CMU-VLA-Challenge

# 다운로드 스크립트 복사
COPY download_leo_models.py /tmp/download_leo_models.py

# LEO 모델 다운로드 실행
RUN /bin/bash -c "source /opt/conda/etc/profile.d/conda.sh && \
    conda activate leo && \
    pip install pydantic-core && \
    pip install protobuf==3.20.3 && \
    python /tmp/download_leo_models.py"

# HuggingFace 캐시 디렉토리 설정
ENV HF_HOME=/root/.cache/huggingface
ENV TRANSFORMERS_CACHE=/root/.cache/huggingface/transformers
ENV PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

# Dockerfile에 추가
VOLUME /root/.cache/huggingface