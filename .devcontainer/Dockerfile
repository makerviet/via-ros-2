FROM athackst/ros2:foxy-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install clang-format libopencv-dev ros-foxy-camera-info-manager \
   ros-foxy-image-transport ros-foxy-rqt-image-view

RUN apt-get install libboost-all-dev protobuf-compiler

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc