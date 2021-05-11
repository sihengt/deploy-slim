# //////////////////////////////////////////////////////////////////////////////
# @brief subt's basic ros dockerfile
# //////////////////////////////////////////////////////////////////////////////

# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM nvidia/opencl:devel-ubuntu18.04

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# //////////////////////////////////////////////////////////////////////////////
# general tools install
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update --no-install-recommends \
 && apt-get install -y \
  build-essential \
  cmake \
  cppcheck \
  gdb \
  git \
  lsb-release \
  software-properties-common \
  sudo \
  vim \
  wget \
  tmux \
  curl \
  less \
  net-tools \
  byobu \
  libgl-dev \
  iputils-ping \
  nano \
  doxygen \
  graphviz \
  python-requests \
  python-pip \
  locales \
  xvfb \
  tzdata \
  emacs \
  # OpenCL libs \
  clinfo \
  cpio \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Dependencies for glvnd and X11.
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    mesa-utils \
  && rm -rf /var/lib/apt/lists/*

# Set the locale
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
 && dpkg-reconfigure --frontend=noninteractive locales \
 && update-locale LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id=1000
ENV USERNAME developer
ENV USER=developer
RUN useradd -U $USERNAME --uid $user_id -ms /bin/bash \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME
# Commands below run as the developer user
USER $USERNAME
# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# TODO
# == warning == this is a security issue
# ssh keys: copy bitbucket ssh keys from host to image
# RUN rm -rf ~/.ssh
# RUN mkdir ~/.ssh
# ARG ssh_priv_key
# ARG ssh_pub_key
# RUN eval "$(ssh-agent -s)" \
#  && sudo chmod 700 ~/.ssh \
#  && sudo chown -R $USERNAME:$USERNAME ~/.ssh \
#  && echo "$ssh_priv_key" >> /home/$USERNAME/.ssh/id_rsa \
#  && echo "$ssh_pub_key" >> /home/$USERNAME/.ssh/id_rsa.pub \
#  && chmod 400 ~/.ssh/id_rsa \
#  && chmod 400 ~/.ssh/id_rsa.pub \
#  && ssh-add ~/.ssh/id_rsa \
#  && ssh-keyscan bitbucket.org >> ~/.ssh/known_hosts

# remove ssh host checking (for git clone in container)
# eventually to be removed by the above ssh keys setup
RUN mkdir $HOME/.ssh && ssh-keyscan bitbucket.org >> $HOME/.ssh/known_hosts

# Set the timezone
RUN sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean

# //////////////////////////////////////////////////////////////////////////////
# ros install
RUN sudo /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && sudo /bin/sh -c 'wget -q http://packages.osrfoundation.org/gazebo.key -O - | APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 sudo apt-key add -' \
 && sudo /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && sudo /bin/sh -c 'apt-key adv --keyserver  hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' \
 && sudo /bin/sh -c 'apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE' \
 && sudo apt-get update \
 && sudo apt-get install -y --no-install-recommends \
  # general ros melodic dependencies \
  python-rosdep \
  libboost-all-dev \
  libeigen3-dev \
  assimp-utils \
  libcgal-dev \
  libcgal-qt5-dev \
  libproj-dev \
  libnlopt-dev \
  python-wstool \
  python-catkin-tools \
  libglfw3-dev \
  libblosc-dev \
  libopenexr-dev \
  liblog4cplus-dev \
  libsuitesparse-dev \
  libsdl1.2-dev \
  # basic ros-melodic packages \
  ros-melodic-catch-ros \
  ros-melodic-smach-viewer \
  ros-melodic-tf-conversions \
  ros-melodic-gazebo-* \
  ros-melodic-random-numbers \
  ros-melodic-cmake-modules \
  ros-melodic-rqt-gui-cpp \
  ros-melodic-rviz \
  ros-melodic-tf2-geometry-msgs \
 && sudo apt-get clean \
 && sudo rm -rf /var/lib/apt/lists/*
# force a rosdep update
RUN sudo rosdep init && rosdep update

# //////////////////////////////////////////////////////////////////////////////
# basic python packages
RUN pip install --user \
 wheel \
 setuptools \
 PyYAML \
 pexpect \
 tmuxp \
 libtmux

RUN sudo add-apt-repository ppa:ubuntu-toolchain-r/test \
&& sudo apt-get update \
&& sudo apt install -y gcc-10 g++-10 \
&& sudo apt-get clean \
&& sudo rm -rf /var/lib/apt/lists/*

# //////////////////////////////////////////////////////////////////////////////
# Install opencl
# -- assumes OpenCL download already exists in the `dockerfiles/thirdparty-software` context path.
# //////////////////////////////////////////////////////////////////////////////

# COPY --chown=$USERNAME:$USERNAME thirdparty-software/ /home/$USERNAME/thirdparty-software/
# RUN cd /home/$USERNAME/thirdparty-software/opencl/ \
#  && tar -xvf l_opencl_p_18.1.0.015.tgz \
#  && cd l_opencl_p_18.1.0.015 \
#  && sudo ./install.sh --silent ../silent-install.cfg --ignore-cpu \
#  && cd ../ \
#  && rm -rf l_opencl_p_18.1.0.015.tgz \
#  && rm -rf l_opencl_p_18.1.0.015 \
#  && clinfo

# COPY --chown=$USERNAME:$USERNAME thirdparty-software/ /home/$USERNAME/thirdparty-software/
# RUN sudo add-apt-repository ppa:intel-opencl/intel-opencl \
#  && sudo apt-get update --no-install-recommends \
#  && sudo apt-get install -y --no-install-recommends \
#   intel-opencl-icd \
#   clinfo

# # add user to video groups for opencl
# RUN sudo usermod -a -G video developer

# # Env vars for the nvidia-container-runtime.
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# # //////////////////////////////////////////////////////////////////////////////
# # export environment variables

# # Ugly: update the python environment path
# ENV PYTHONPATH=${PYTHONPATH}:/home/$USERNAME/.local/lib/python2.7/site-packages/
# ENV PATH=${PATH}:/home/$USERNAME/.local/bin/

# # //////////////////////////////////////////////////////////////////////////////
# # entrypoint startup
# # //////////////////////////////////////////////////////////////////////////////

# # entrypoint path inside the docker container
# ENV entry_path /docker-entrypoint/

# # add entrypoint scripts (general & system specific)
# ADD entrypoints/ $entry_path/

# # execute entrypoint script
# RUN sudo chown -R $USERNAME:$USERNAME $entry_path/
# RUN sudo chmod +x -R $entry_path/

# # set image to run entrypoint script
# ENTRYPOINT $entry_path/docker-entrypoint.bash
