FROM ubuntu:focal
RUN apt update -y && apt install curl gnupg2 lsb-release -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt install wget -y
RUN wget https://github.com/ros2/ros2/releases/download/release-foxy-20210902/ros2-foxy-20210902-linux-focal-amd64.tar.bz2
WORKDIR /robot_software_transformers-main
COPY . .
RUN mkdir -p /ros2_foxy
WORKDIR /ros2_foxy
RUN tar -xf /ros2-foxy-20210902-linux-focal-amd64.tar.bz2
RUN apt update -y
RUN apt install -y python3-rosdep
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths /ros2_foxy/ros2-linux/share --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps osrf_testing_tools_cpp poco_vendor rmw_connextdds rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"
RUN apt install -y libpython3-dev python3-pip
RUN pip3 install -U argcomplete
WORKDIR /robot_software_transformers-main
RUN mkdir src
RUN cp package.xml src/
RUN cp -r resource src/
RUN cp -r robot_software_transformers src/
RUN cp setup.cfg src/
RUN cp setup.py src/
RUN cp -r test src/
ENTRYPOINT /bin/bash