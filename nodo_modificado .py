import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float32MultiArray

class RPMReaderNode(Node):
    def __init__(self):
        super().__init__('rpm_reader_node')
        self.get_logger().info('Nodo de lectura de RPM iniciado')

        # Conexión al puerto serie del Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)  # Esperar a que la conexión se establezca
            self.get_logger().info('Conexión serial establecida')
        except serial.SerialException as e:
            self.get_logger().error(f'Error de conexión serial: {str(e)}')
            exit(1)

        # Publicador para enviar RPM y velocidad
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'rpm_data', 10)
        
        # Timer para lectura periódica cada 2 segundos
        self.timer = self.create_timer(2.0, self.read_rpm)

    def read_rpm(self):
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Datos recibidos: {data}")

                # Procesar datos (suponiendo formato "RPM_right,RPM_left")
                rpm_values = data.split(',')
                if len(rpm_values) == 2:
                    rpm_right = float(rpm_values[0])
                    rpm_left = float(rpm_values[1])

                    # Cálculo de velocidad lineal: v = (RPM * π * D) / 60
                    wheel_diameter = 0.2032  # 203.2 mm = 0.2032 m
                    pi = 3.14159265359
                    speed_right = (rpm_right * pi * wheel_diameter) / 60.0
                    speed_left = (rpm_left * pi * wheel_diameter) / 60.0

                    # Publicar los datos en el tópico de ROS 2
                    msg = Float32MultiArray()
                    msg.data = [rpm_right, rpm_left, speed_right, speed_left]
                    self.rpm_publisher.publish(msg)

                    self.get_logger().info(f"RPM Derecha: {rpm_right}, RPM Izquierda: {rpm_left}")
                    self.get_logger().info(f"Velocidad Derecha: {speed_right:.2f} m/s, Velocidad Izquierda: {speed_left:.2f} m/s")

        except Exception as e:
            self.get_logger().error(f"Error al leer desde serial: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    rpm_reader_node = RPMReaderNode()
    rclpy.spin(rpm_reader_node)
    rpm_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
