FROM ros:noetic-ros-base-focal

USER 0

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y cmake iputils-ping tftp lsb-core wget && \
    apt-get install -y libeigen3-dev && \
    apt-get install -y libopencv-dev && \
    apt-get install -y libgoogle-glog-dev libgflags-dev && \
    apt-get install -y libatlas-base-dev && \
    apt-get install -y libsuitesparse-dev && \
    apt-get install -y zsh

RUN DEBIAN_FRONTEND=noninteractive apt-get install lightdm -y && \
    apt-get install -y libpcl-dev && \
    apt-get install -y libceres-dev && \
    apt-get install -y libprotobuf-dev && \
    apt-get clean
    
RUN apt-get install -y sudo vim && \
    apt-get install -y ros-noetic-pcl-conversions && \
    apt-get install -y ros-noetic-tf\* && \
    apt-get install -y ros-noetic-image-transport-plugins && \
    apt-get install -y openssh-server && \
    apt-get install -y git && \
    apt-get install -y libyaml-cpp-dev

RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

RUN useradd -m test && echo "test:test" | chpasswd && adduser test sudo

USER test

RUN cd /home/test && \
    sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"
    

RUN mkdir -p /home/test/workspace/src && \
    cd /home/test/workspace

    
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && cd /home/test/workspace && catkin_make"

RUN echo "source /opt/ros/noetic/setup.zsh" >> /home/test/.zshrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/test/.bashrc && \
    echo "source /home/test/workspace/devel/setup.zsh" >> /home/test/.zshrc && \
    echo "source /home/test/workspace/devel/setup.bash" >> /home/test/.bashrc



    
