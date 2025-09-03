FROM osrf/ros:jazzy-desktop-full

# Install gz

RUN apt install -y curl lsb-release gnupg && \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update && apt-get install -y gz-harmonic && \
    apt install -y ros-jazzy-cartographer ros-jazzy-cartographer-ros ros-jazzy-navigation2 ros-jazzy-nav2-bringup net-tools iputils-ping tmux

# Crear workspace dentro del contenedor
WORKDIR /ros2_ws

# Copiar tu carpeta src local al contenedor
COPY ./ros2_ws/src ./src

# Construir los paquetes
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Configurar sourcing automático al entrar al contenedor
ENV TURTLEBOT3_MODEL=burger

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=1 #TURTLEBOT3' >> ~/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    export TURTLEBOT3_MODEL=burger

RUN bash -c "source ~/.bashrc"

# Entrar al contenedor en modo bash
CMD ["/bin/bash"]