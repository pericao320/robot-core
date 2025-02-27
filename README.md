# RPM Reader Node

Este proyecto contiene un nodo de ROS 2 llamado `RPMReaderNode` que se encarga de leer datos de RPM desde un Arduino conectado a través de un puerto serie y publicar estos datos junto con las velocidades calculadas en un tópico de ROS 2.

## Requisitos

- ROS 2 (Jazzy)
- Python 3.6 o superior
- Biblioteca `pyserial`
- Arduino con la biblioteca `Servo`
- Raspberry Pi password: Robusto2025

## Instalación

1. Clona este repositorio y muevelo a tu espacio de trabajo de ROS 2:
    ```sh
    git clone --no-checkout <URL_DEL_REPOSITORIO> ~/repo_temp
    cd ~/repo_temp
    git sparse-checkout init --cone
    git sparse-checkout set ruta/al/archivo/nodo_modificado.py
    git checkout

    mv ruta/al/archivo/nodo_modificado.py ~/ros2_ws/src/
    rm -rf ~/repo_temp
    ```

2. Instala la biblioteca `pyserial` si no la tienes instalada:
    ```sh
    pip install pyserial
    ```

## Uso

### Nodo de ROS 2

1. Conecta tu Arduino al puerto serie de tu computadora.

2. Asegúrate de que el puerto serie en el código (`/dev/ttyACM0`) coincide con el puerto al que está conectado tu Arduino. Si estás en Windows, el puerto puede ser algo como `COM3`.

3. Compila tu espacio de trabajo de ROS 2:
    ```sh
    colcon build
    ```

4. Fuente el entorno de tu espacio de trabajo:
    ```sh
    source install/setup.bash
    ```

5. Ejecuta el nodo:
    ```sh
    ros2 run robot_rpm_monitor rpm_reader_node
    ```

## Descripción del Código

### Nodo de ROS 2 (`nodo_modificado.py`)

El archivo `nodo_modificado.py` contiene la implementación del nodo `RPMReaderNode`. Aquí hay una breve descripción de las partes principales del código:

- **Inicialización del Nodo**: El nodo se inicializa y se establece una conexión con el puerto serie del Arduino.
- **Publicador**: Se crea un publicador para enviar los datos de RPM y velocidad en el tópico `rpm_data`.
- **Timer**: Se configura un temporizador para leer los datos del puerto serie cada 2 segundos.
- **Lectura de RPM**: La función `read_rpm` lee los datos del puerto serie, los procesa y publica los valores de RPM y velocidad.

### Código de Arduino (`codigo_arduino_comunicacionros2_bueno.ino`)

El archivo [codigo_arduino_comunicacionros2_bueno.ino](http://_vscodecontentref_/3) contiene la implementación del control de motores y la lectura de encoders. Aquí hay una breve descripción de las partes principales del código:

- **Configuración de Motores y Encoders**: Se configuran los pines para los motores y los encoders, y se establecen las interrupciones para contar los pulsos de los encoders.
- **Cálculo de RPM**: La función [calculateRPM](http://_vscodecontentref_/4) calcula las RPM de los motores cada 2 segundos y envía los datos al nodo de ROS 2 a través del puerto serie.
- **Control de Motores**: El código recibe comandos a través de Bluetooth para controlar los motores (avanzar, retroceder, detener, girar, etc.).

## Contribuciones

Las contribuciones son bienvenidas. Por favor, abre un issue o un pull request para discutir cualquier cambio que te gustaría hacer.
## Anexos
### Descarga el apk
<img src="https://i.imgur.com/FSFp2WL.jpeg" title="Imagen de ejemplo (Simula una capturada por el sensor Kinect)" alt="RvizImage" height="250"/>