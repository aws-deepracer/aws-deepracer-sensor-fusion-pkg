#!/usr/bin/env python

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

# '''
# Instructions to use the test node:

# SSH into the device in three different terminals.
# Run the following in one of the terminal.

# systemctl stop deepracer-core.service

# Terminal 1:
# Start the roscore

#   source /opt/ros/kinetic/setup.bash

#   source /opt/aws/deepracer/setup.bash

#   source /opt/aws/intel/dldt/bin/setupvars.sh

#   roscore

# Terminal 2:
    
# Start the sensor_fusion_node
#  sudo su
 
#  source /opt/ros/kinetic/setup.bash
 
#  source /opt/aws/deepracer/setup.bash
 
#  source /opt/aws/intel/dldt/bin/setupvars.sh

#  cd /opt/aws/deepracer/lib/sensor_fusion_pkg

#  rosrun sensor_fusion_pkg sensor_fusion_node.py

# Terminal 3: 
#  sudo su
 
#  source /opt/ros/kinetic/setup.bash
 
#  source /opt/aws/deepracer/setup.bash
 
#  source /opt/aws/intel/dldt/bin/setupvars.sh

#  cd /opt/aws/deepracer/lib/sensor_fusion_pkg

# Edit the test node to set the model path or path to model xml.

#  rosrun sensor_fusion_pkg test_node.py

# Terminal 4:

# If you want to test the topic sensor_msg with camera msg
#  sudo su
 
#  source /opt/ros/kinetic/setup.bash
 
#  source /opt/aws/deepracer/setup.bash
 
#  source /opt/aws/intel/dldt/bin/setupvars.sh

#  cd /opt/aws/deepracer/lib/sensor_fusion_pkg

# Edit the test node to set the model path or path to model xml.

#  rosrun media_pkg media_node

# Terminal 5: 
# If you want to test the topic sensor_msg with lidar
#  sudo su
 
#  source /opt/ros/kinetic/setup.bash
 
#  source /opt/aws/deepracer/setup.bash
 
#  source /opt/aws/intel/dldt/bin/setupvars.sh

#  cd /opt/aws/deepracer/lib/sensor_fusion_pkg

# Edit the test node to set the model path or path to model xml.

#  rosrun rplidar_ros rplidarRosNode
# '''

# import cv2
# from sensor_msgs.msg import LaserScan
# from sensor_msgs.msg import Image
# # from cv_bridge import CvBridge, CvBridgeError
# from media_pkg.msg import cameraMSG
# from sensor_fusion_pkg.msg import sensorMSG
# from sensor_fusion_pkg.srv import LidarConfigSrv, SensorStatusCheckSrv

# import rospy

# image_counter = 0

# left_image_path = '/home/deepracer/Desktop/left-image.jpg'
# right_image_path = '/home/deepracer/Desktop/right-image.jpg'
# configure_lidar_values = False
# fetch_sensor_status = False
# dump_data_to_directory = False
# publish_custom_data = False

# image_counter = 0

# # vary this to check the output
# max_lidar_range = 10.0
# min_lidar_range = 0.15

# # Default configure values
# use_lidar = False
# min_lidar_angle = -60.0
# max_lidar_angle = 60.0
# min_lidar_dist = 0.0
# max_lidar_dist = 1.0
# lidar_clipping_dist = 1.0
# num_lidar_values = 64

# def sensor_fusion_cb(data):
#     global image_counter
#     global dump_data_to_directory
#     global fetch_sensor_status
#     global sensor_status
#     print(data.lidar_data)
#     print()
#     bridge = CvBridge()

#     if dump_data_to_directory:
#         for i, img in enumerate(data.images):
#             try:
#                 cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
#             except CvBridgeError as e:
#                  print(e)
#             image_ = cv2.resize(cv_image, (160, 120))
#             image_name = 'image-{}-{}'.format(i, image_counter)

#             cv2.imwrite('/home/deepracer/sensor_fusion_data/{}.jpg'.format(image_name), image_)

#     image_counter += 1

#     if fetch_sensor_status:
#         sensor_resp = sensor_status()
#         print("Camera status: {0}, Stereo status: {1}, Lidar status: {2}".format(sensor_resp.single_camera_status, \
#                                                                  sensor_resp.stereo_camera_status,
#                                                                  sensor_resp.lidar_status))

# rospy.init_node('test_node', anonymous=True)
# if configure_lidar_values == True:
#     rospy.wait_for_service('configure_lidar')
# if fetch_sensor_status == True:
#     rospy.wait_for_service('sensor_data_status')

# sensor_status = rospy.ServiceProxy('sensor_data_status', SensorStatusCheckSrv)

# if configure_lidar_values == True:
#     configure_lidar = rospy.ServiceProxy('configure_lidar', LidarConfigSrv)
#     configure_lidar_resp = configure_lidar(use_lidar, 
#                          min_lidar_angle, 
#                          max_lidar_angle, 
#                          num_lidar_values, 
#                          min_lidar_dist, 
#                          max_lidar_dist, 
#                          lidar_clipping_dist)
#     if configure_lidar_resp.hasError:
#         print("Configure lidar failed")
#         exit()


# rospy.Subscriber("sensor_msg", sensorMSG, sensor_fusion_cb)

# cv_image_left = cv2.imread(left_image_path,1)
# cv_image_right = cv2.imread(right_image_path, 1)
# lidar_data_ranges = [float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 8.628000259399414, 8.35200023651123, 7.793000221252441, float('inf'), float('inf'), 7.006999969482422, 6.742000102996826, 6.394999980926514, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 4.178999900817871, 4.1529998779296875, 4.01800012588501, 3.98799991607666, 3.9119999408721924, 3.8269999027252197, 3.7839999198913574, 3.7269999980926514, 3.6689999103546143, 3.6689999103546143, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 3.0880000591278076, 3.1710000038146973, 3.178999900817871, 7.440000057220459, 7.383999824523926, 7.297999858856201, 7.204999923706055, 7.146999835968018, 7.113999843597412, float('inf'), float('inf'), float('inf'), 7.015999794006348, 6.942999839782715, 6.9120001792907715, 6.872000217437744, 6.875999927520752, float('inf'), 2.9210000038146973, 2.7809998989105225, 2.7709999084472656, 2.7709999084472656, 2.765000104904175, 2.76200008392334, 2.760999917984009, 2.8469998836517334, 2.9140000343322754, 3.000999927520752, 3.2179999351501465, 3.509999990463257, 3.6410000324249268, 8.378000259399414, 8.364999771118164, 8.451000213623047, 8.33899974822998, 8.404000282287598, 8.364999771118164, 8.41100025177002, 8.416999816894531, 8.498000144958496, 8.503999710083008, 8.552000045776367, 8.498000144958496, 8.552000045776367, 2.816999912261963, 2.8570001125335693, float('inf'), float('inf'), float('inf'), 3.0840001106262207, 3.0889999866485596, 3.128999948501587, float('inf'), float('inf'), 3.2269999980926514, 3.239000082015991, float('inf'), float('inf'), float('inf'), 3.321000099182129, 3.2980000972747803, 3.2130000591278076, 3.2669999599456787, 3.303999900817871, 3.365000009536743, 3.4010000228881836, 3.434999942779541, 3.4590001106262207, 3.5460000038146973, 3.5999999046325684, 3.6080000400543213, 3.7119998931884766, 3.752000093460083, 3.819999933242798, 3.875999927520752, 3.933000087738037, 3.9749999046325684, 4.089000225067139, 4.193999767303467, 4.247000217437744, 4.382999897003174, 4.440999984741211, 4.589000225067139, 4.6529998779296875, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 8.41100025177002, 8.503999710083008, 8.312999725341797, 3.75600004196167, 3.7679998874664307, 3.746999979019165, 3.7290000915527344, float('inf'), float('inf'), 0.5230000019073486, 0.5170000195503235, 0.5149999856948853, 0.5149999856948853, 0.5120000243186951, 0.5120000243186951, 0.5099999904632568, 0.5090000033378601, 0.5080000162124634, 0.5070000290870667, 0.5070000290870667, 0.5059999823570251, 0.5049999952316284, 0.5059999823570251, 0.5049999952316284, 0.5139999985694885, float('inf'), 0.3440000116825104, 0.3330000042915344, 0.33899998664855957, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 0.3269999921321869, float('inf'), 0.32199999690055847, 0.32199999690055847, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 0.34700000286102295, 0.35600000619888306, 0.39500001072883606, float('inf'), 0.5830000042915344, 0.5879999995231628, 0.597000002861023, 0.6039999723434448, 0.6079999804496765, 0.6119999885559082, 0.621999979019165, 0.6259999871253967, 0.6380000114440918, 0.6449999809265137, 0.656000018119812, 0.6629999876022339, 0.6740000247955322, 0.6830000281333923, 0.6970000267028809, 0.7139999866485596, 0.7229999899864197, 0.7379999756813049, 0.7480000257492065, 0.7699999809265137, 0.7770000100135803, 0.7879999876022339, 0.8119999766349792, 0.8330000042915344, 0.847000002861023, float('inf'), float('inf'), 0.4180000126361847, 0.41499999165534973, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 2.8329999446868896, float('inf'), 2.99399995803833, 2.234999895095825, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 0.20999999344348907, 0.20800000429153442, 0.2070000022649765, 0.20399999618530273, 0.2029999941587448, 0.20100000500679016, 0.19900000095367432, 0.19699999690055847, 0.19599999487400055, 0.19599999487400055, 0.1940000057220459, 0.19300000369548798, 0.19200000166893005, float('inf'), 0.1899999976158142, 0.1889999955892563, 0.18799999356269836, 0.18799999356269836, 0.18700000643730164, 0.1860000044107437, 0.1850000023841858, 0.1850000023841858, 0.18400000035762787, 0.18400000035762787, 0.18400000035762787, 0.18400000035762787, 0.18400000035762787, 0.18299999833106995, 0.18299999833106995, 0.18299999833106995, 0.18400000035762787, 0.18299999833106995, 0.18299999833106995, 0.18299999833106995, 0.18400000035762787, 0.18400000035762787, 0.18400000035762787, 0.1850000023841858, 0.1850000023841858, 0.1860000044107437, 0.1860000044107437, 0.18700000643730164, 0.18799999356269836, 0.1889999955892563, 0.1889999955892563, 0.1899999976158142, 0.19200000166893005, 0.19300000369548798, 0.19300000369548798, 0.1940000057220459, float('inf'), 0.19699999690055847, 0.19900000095367432, 0.20200000703334808, 0.20499999821186066, 0.2070000022649765, 0.210999995470047, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 0.21699999272823334, float('inf'), 0.22300000488758087, 0.2280000001192093, 0.23000000417232513, 0.23999999463558197, float('inf'), 0.2409999966621399, 0.24300000071525574, 0.25099998712539673, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 2.680999994277954, 2.632999897003174, 3.7669999599456787, 3.7639999389648438, 3.7330000400543213, 3.7330000400543213, 2.6989998817443848, 2.7070000171661377, 2.683000087738037, 2.7119998931884766, 3.697999954223633, 3.6059999465942383, 3.697999954223633, 3.7119998931884766, float('inf'), float('inf'), float('inf')]
# media_node_pub = rospy.Publisher("video_mjpeg", cameraMSG, queue_size=1)
# lidar_pub = rospy.Publisher("scan", LaserScan, queue_size=1)

# bridge = CvBridge()

# while not rospy.is_shutdown():
#     try:
#         if publish_custom_data:
#             msg = cameraMSG()
#             msg.images = [bridge.cv2_to_imgmsg(cv_image_left, "bgr8"), bridge.cv2_to_imgmsg(cv_image_right, "bgr8")]
#             lidar_values = LaserScan()
#             lidar_values.angle_min = -3.14159274101
#             lidar_values.angle_max = 3.14159274101
#             lidar_values.angle_increment = 0.0175019092858
#             lidar_values.time_increment = 0.000400334451115
#             lidar_values.scan_time = 0.143720060587
#             lidar_values.range_min = 0.15000000596
#             lidar_values.range_max = 12.0
#             lidar_values.ranges = lidar_data_ranges
#             lidar_values.intensities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 0.0, 0.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 0.0, 0.0, 47.0, 47.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 0.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 0.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 0.0, 47.0, 47.0, 47.0, 47.0, 0.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 47.0, 0.0, 0.0, 0.0]
#             media_node_pub.publish(msg)
#             lidar_pub.publish(lidar_values)
#     except CvBridgeError as e:
#         print(e)
