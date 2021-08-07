FROM athackst/ros2:foxy-dev 

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install clang-format libopencv-dev ros-foxy-camera-info-manager \
   ros-foxy-image-transport ros-foxy-rqt-image-view

RUN apt-get install -y libboost-all-dev protobuf-compiler

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrcdocke