# Workshop RAS - Robótica y Navegación con ROS2

## 📋 Resumen

Este proyecto es un taller de robótica enfocado en el aprendizaje de ROS2 (Robot Operating System 2) utilizando TurtleBot3 como plataforma de desarrollo. El workshop incluye simulaciones en Gazebo y control de robots móviles.

### 🎯 Objetivos del Workshop

- Aprender los fundamentos de ROS2
- Configurar y controlar un robot TurtleBot3
- Desarrollar habilidades en simulación robótica con Gazebo

### 🔧 Tecnologías Utilizadas

- **ROS2 Jazzy**: Framework de robótica
- **Gazebo Harmonic**: Simulador 3D
- **TurtleBot3**: Plataforma de robot móvil
- **DynamixelSDK**: Control de servomotores Dynamixel
- **Docker**: Contenedorización del entorno

## 🚀 Instalación y Configuración

### Prerrequisitos

Antes de comenzar, asegúrate de tener instalado en tu sistema Linux:

- Docker
- Docker Compose (opcional)
- Sistema X11 para la interfaz gráfica

#### Instalación de Docker en Ubuntu/Debian

```bash
# Actualizar el sistema
sudo apt update

# Instalar dependencias
sudo apt install apt-transport-https ca-certificates curl gnupg lsb-release

# Agregar la clave GPG oficial de Docker
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Agregar el repositorio de Docker
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Instalar Docker
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io

# Agregar tu usuario al grupo docker (opcional, para evitar usar sudo)
sudo usermod -aG docker $USER
```

Después de agregar tu usuario al grupo docker, cierra sesión y vuelve a iniciarla.

### 📥 Instalación del Proyecto

1. **Clonar el repositorio:**
   ```bash
   git clone https://github.com/miguelgonrod/Workshop-RAS-RNR.git
   cd Workshop-RAS-RNR
   ```

2. **Dar permisos de ejecución al script:**
   ```bash
   chmod +x run.sh
   ```

3. **Construir y ejecutar el contenedor:**
   ```bash
   ./run.sh
   ```

### 🔧 Construcción Manual (Alternativa)

Si prefieres construir y ejecutar manualmente:

```bash
# Construir la imagen Docker
docker build -t ros2-rnr .

# Ejecutar el contenedor con soporte gráfico
xhost +local:docker
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros2-rnr
```

## 🎮 Guía de Uso

### Inicialización del Entorno

Una vez dentro del contenedor, el workspace de ROS2 ya está configurado. Para comenzar:

```bash
# Cargar el entorno de ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 🤖 Simulación con TurtleBot3

#### 1. Lanzar la simulación básica

```bash
# Configurar el modelo de TurtleBot3
export TURTLEBOT3_MODEL=burger

# Lanzar Gazebo con TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### 2. Control manual del robot

En una nueva terminal (dentro del contenedor):

```bash
# Teleoperación con teclado
ros2 run turtlebot3_teleop teleop_keyboard
```

### 🔧 Desarrollo Personalizado

El paquete `taller_rnr` está incluido para desarrollo personalizado:

```bash
# Navegar al workspace
cd /ros2_ws

# Construir el paquete personalizado
colcon build --packages-select taller_rnr

# Cargar el entorno actualizado
source install/setup.bash
```

## 📁 Estructura del Proyecto

```
Workshop-RAS-RNR/
├── Dockerfile              # Configuración del contenedor
├── run.sh                  # Script de ejecución simplificado
├── README.md              # Este archivo
└── ros2_ws/               # Workspace de ROS2
    └── src/
        ├── DynamixelSDK/  # SDK para motores Dynamixel
        ├── taller_rnr/    # Paquete personalizado del taller
        ├── turtlebot3/    # Paquetes del TurtleBot3
        ├── turtlebot3_msgs/        # Mensajes del TurtleBot3
        └── turtlebot3_simulations/ # Simulaciones del TurtleBot3
```

## 🛠️ Solución de Problemas

### Problemas de Visualización

Si no puedes ver las ventanas gráficas (Gazebo, RViz):

```bash
# Verificar que X11 forwarding esté habilitado
echo $DISPLAY

# Reiniciar el permiso X11
xhost +local:docker
```

### Problemas de Red

Si hay problemas de conectividad entre nodos:

```bash
# Verificar que la red esté configurada correctamente
docker run --net=host ...
```

### Reconstruir el Proyecto

Si necesitas reconstruir después de cambios:

```bash
# Dentro del contenedor
cd /ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## 📚 Recursos Adicionales

- [Documentación oficial de ROS2](https://docs.ros.org/en/jazzy/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Navigation2 Documentation](https://navigation.ros.org/)

## 🤝 Contribuciones

Las contribuciones son bienvenidas. Por favor:

1. Fork el proyecto
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 📄 Licencia

Este proyecto está bajo la licencia BSD 3-Clause. Ver el archivo `LICENSE` para más detalles.

## 👨‍💻 Autores

**Miguel González** - [miguelgonrod](https://github.com/miguelgonrod) <br>
**Iván Orozco** - [Nidhood](https://github.com/Nidhood)

---

*¡Disfruta aprendiendo robótica con ROS2! 🤖*

<img width="1600" height="151" alt="image" src="https://github.com/user-attachments/assets/9d9610d0-d2bf-40aa-92e8-11745bc57ab1" />
