"""
robot_launch.py  -  Lanza todo el sistema MAESTRO en la Raspberry Pi 5:

  * robot_core         (driver motores + odometria + PID)
  * cmd_vel_mux        (prioriza teleop > auto > player)
  * robot_player       (reproductor de trayectorias en lazo cerrado)
  * robot_interface    (GUI de grabacion/reproduccion/visualizacion)
  * vision_node        (deteccion de cubos de color)
  * forklift_manager   (FSM autonomo del montacargas)
  * esp32_bridge       (serial USB a la ESP32)
  * joy_node           (joystick bluetooth)
  * teleop_twist_joy   (traduce Joy a Twist, publica en /cmd_vel_teleop)

La interfaz grafica y los teleop opcionales se lanzan aparte porque
requieren terminal/entorno X.
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    serial_port = DeclareLaunchArgument(
        'esp32_port', default_value='/dev/ttyUSB0',
        description='Puerto serie de la ESP32')

    return LaunchDescription([
        serial_port,

        Node(package='diff_bot_3', executable='robot_core',
             name='robot_core'),

        Node(package='diff_bot_3', executable='cmd_vel_mux',
             name='cmd_vel_mux'),

        Node(package='diff_bot_3', executable='robot_player',
             name='robot_player'),

        Node(package='diff_bot_3', executable='vision_node',
             name='vision_node'),

        Node(package='diff_bot_3', executable='forklift_manager',
             name='forklift_manager'),

        Node(package='diff_bot_3', executable='esp32_bridge',
             name='esp32_bridge',
             parameters=[{'port': LaunchConfiguration('esp32_port'),
                          'baud': 115200}]),

        # ---- Joystick bluetooth -> /cmd_vel_teleop -----------------------
        Node(package='joy', executable='joy_node', name='joy_node'),

        Node(package='teleop_twist_joy',
             executable='teleop_node',
             name='teleop_twist_joy',
             parameters=[{
                 'require_enable_button': False,
                 'axis_linear.x':   1,
                 'axis_angular.yaw': 0,
                 'scale_linear.x':   0.25,      # bateria 2S -> menos vel. util
                 'scale_angular.yaw': 1.20,
             }],
             remappings=[('/cmd_vel', '/cmd_vel_teleop')]),
    ])
