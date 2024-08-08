import rospy
from std_msgs.msg import Float32MultiArray, String
from pupil_labs.realtime_api.simple import discover_one_device

def pupil_size_publisher():
    rospy.init_node('pupil_size_publisher', anonymous=True)
    device_info_pub = rospy.Publisher('pupil_size_device_info', String, queue_size=10)
    pupil_diameter_pub = rospy.Publisher('pupil_diameter', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    device = discover_one_device()
    
    # Publish device info
    device_info = (f"Phone IP address: {device.phone_ip}\n"
                   f"Phone name: {device.phone_name}\n"
                   f"Battery level: {device.battery_level_percent}%\n"
                   f"Free storage: {device.memory_num_free_bytes / 1024**3:.1f} GB\n"
                   f"Serial number of connected glasses: {device.serial_number_glasses}")
    device_info_pub.publish(device_info)

    try:
        while not rospy.is_shutdown():
            gaze_sample = device.receive_gaze_datum()
            pupil_diameter = Float32MultiArray()
            pupil_diameter.data = [gaze_sample.pupil_diameter_left, gaze_sample.pupil_diameter_right]
            pupil_diameter_pub.publish(pupil_diameter)
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Stream stopped")
    finally:
        device.close()
        rospy.loginfo("Device closed")

if __name__ == '__main__':
    try:
        pupil_size_publisher()
    except rospy.ROSInterruptException:
        pass
