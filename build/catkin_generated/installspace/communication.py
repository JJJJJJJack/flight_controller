import rospy
from rospy.node import Node
import serial
import struct
from std_msgs.msg import Float64

class WindSensorNode(Node):
    def __init__(self):
        super().__init__('wind_sensor_node')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 4800, timeout=1)
        self.windSpeedPublisher_ = self.create_publisher(Float64, 'windSpeed', 10)
        self.windAnglePublisher_ = self.create_publisher(Float64, 'windAngle', 10)
        self.timer = self.create_timer(0.1, self.read_sensor_data)

    def read_sensor_data(self):
        wind_speed_cmd = bytes([0x01, 0x03, 0x01, 0xF4, 0x00, 0x01, 0xC4, 0x04])
        wind_direction_cmd = bytes([0x01, 0x03, 0x01, 0xF7, 0x00, 0x01, 0x34, 0x04])
        
        self.serial_port.write(wind_speed_cmd)
        wind_speed_response = self.serial_port.read(7)
        wind_speed = self.parse_response(wind_speed_response)
        #self.get_logger().info(f"Wind Speed: {wind_speed}")
        msg = Float64()
        msg.data = wind_speed/100
        self.windSpeedPublisher_.publish(msg)

        self.serial_port.write(wind_direction_cmd)
        wind_direction_response = self.serial_port.read(7)
        wind_direction = self.parse_response(wind_direction_response)
        #self.get_logger().info(f"Wind Direction: {wind_direction}")
        msg.data = wind_direction
        self.windAnglePublisher_.publish(msg)

    def parse_response(self, response):
        if len(response) == 7:
            # 假设有效数据在第4和第5位，将其转换为十进制
            return struct.unpack('>H', response[3:5])[0]
        else:
            self.get_logger().warn(f"Invalid response length: {len(response)}")
            return None

def main(args=None):
    rospy.init(args=args)
    node = WindSensorNode()
    rospy.spin(node)
    node.destroy_node()
    rospy.shutdown()

if __name__ == '__main__':
    main()
