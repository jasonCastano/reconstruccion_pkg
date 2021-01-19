#!/usr/bin/env python
import roslib
roslib.load_manifest('subscribers')
import rospy
import sys
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import argparse
import numpy as np
import os

def main(args):
    bridge = CvBridge()

    d1_rooth = "/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D1_outputs/D1"
    d3_rooth = "/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D3_outputs/D3"
    d4_rooth = "/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D4_outputs/D4"
    d5_rooth = "/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D5_outputs/D5"

    d1_rawimgPub = rospy.Publisher("depth/image_rect_d1",Image, queue_size=1)
    d1_colimgPub = rospy.Publisher("rgb/image_rect_color_d1", Image, queue_size=1)

    d3_rawimgPub = rospy.Publisher("depth/image_rect_d3",Image, queue_size=1)
    d3_colimgPub = rospy.Publisher("rgb/image_rect_color_d3", Image, queue_size=1)

    d4_rawimgPub = rospy.Publisher("depth/image_rect_d4",Image, queue_size=1)
    d4_colimgPub = rospy.Publisher("rgb/image_rect_color_d4", Image, queue_size=1)

    d5_rawimgPub = rospy.Publisher("depth/image_rect_d5",Image, queue_size=1)
    d5_colimgPub = rospy.Publisher("rgb/image_rect_color_d5", Image, queue_size=1)
    

    d1_color_camInfoPub = rospy.Publisher("rgb/camera_info_d1",CameraInfo,queue_size=1)
    d1_depth_camInfoPub = rospy.Publisher("depth/camera_info_d1",CameraInfo,queue_size=1)

    d3_color_camInfoPub = rospy.Publisher("rgb/camera_info_d3",CameraInfo,queue_size=1)
    d3_depth_camInfoPub = rospy.Publisher("depth/camera_info_d3",CameraInfo,queue_size=1)

    d4_color_camInfoPub = rospy.Publisher("rgb/camera_info_d4",CameraInfo,queue_size=1)
    d4_depth_camInfoPub = rospy.Publisher("depth/camera_info_d4",CameraInfo,queue_size=1)

    d5_color_camInfoPub = rospy.Publisher("rgb/camera_info_d5",CameraInfo,queue_size=1)
    d5_depth_camInfoPub = rospy.Publisher("depth/camera_info_d5",CameraInfo,queue_size=1)

    #CAMARA D1

    d1_color_camInfo = CameraInfo()
    d1_color_camInfo.header.frame_id = "d1_frame"
    d1_color_camInfo.height = 1080
    d1_color_camInfo.width = 1920
    d1_color_camInfo.distortion_model = "plumb_bob"

    d1_depth_camInfo = CameraInfo()
    d1_depth_camInfo.header.frame_id = "d1_frame"
    d1_depth_camInfo.height = 424
    d1_depth_camInfo.width = 512
    d1_depth_camInfo.distortion_model = "plumb_bob"


    d1_color_camInfo.D = [0.108, -0.125, 0.062, -0.001, -0.003]
    d1_color_camInfo.K = [1144.361,0.00000,966.359,0.00000,1147.337,548.038,0.00000,0.00000,1.00000]
    d1_color_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d1_color_camInfo.P = [1144.361,0.00000,966.359,0.0,0.00000,1147.337,548.038,0.0,0.00000,0.00000,1.00000,0.0]

    # d1_color_camInfo.K = [1064.55179,0.00000,977.84875,0.00000,1063.90832,511.76702,0.00000,0.00000,1.00000]
    # d1_color_camInfo.R = [0.99993,-0.01203,-0.00000,-0.01203,-0.99993,-0.00000,0.00000,-0.00000,1.00000]
    # d1_color_camInfo.P = [1064.46978,-13.21356,977.84875,56231.34272,-12.39934,-1063.83606,511.76702,-0.04158,0.00000,-0.00000,1.00000,0.00000]

    d1_depth_camInfo.D = [0.126, -0.329, 0.111, -0.001, -0.002]
    d1_depth_camInfo.K = [388.198,0.00000,253.270,0.00000,389.033,213.934,0.00000,0.00000,1.00000]
    d1_depth_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d1_depth_camInfo.P = [388.198,0.00000,253.270,0.0,0.00000,389.033,213.934,0.0,0.00000,0.00000,1.00000,0.0]

    #CAMARA D3

    d3_color_camInfo = CameraInfo()
    d3_color_camInfo.header.frame_id = "d3_frame"
    d3_color_camInfo.height = 1080
    d3_color_camInfo.width = 1920
    d3_color_camInfo.distortion_model = "plumb_bob"

    d3_depth_camInfo = CameraInfo()
    d3_depth_camInfo.header.frame_id = "d3_frame"
    d3_depth_camInfo.height = 424
    d3_depth_camInfo.width = 512
    d3_depth_camInfo.distortion_model = "plumb_bob"


    d3_color_camInfo.D = [0.108, -0.125, 0.062, -0.001, -0.003]
    d3_color_camInfo.K = [1144.361,0.00000,966.359,0.00000,1147.337,548.038,0.00000,0.00000,1.00000]
    d3_color_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d3_color_camInfo.P = [1144.361,0.00000,966.359,0.0,0.00000,1147.337,548.038,0.0,0.00000,0.00000,1.00000,0.0]

    # d3_color_camInfo.K = [1060.94487,0.00000,966.41930,0.00000,1061.28267,543.90629,0.00000,0.00000,1.00000]
    # d3_color_camInfo.R = [0.99990,-0.01414,-0.00000,-0.01414,-0.99990,0.00000,0.00000,0.00000,1.00000]
    # d3_color_camInfo.P = [1060.83871,-15.00812,966.41930,56231.34341,-14.99981,-1061.17667,543.90629,0.15523,0.00000,0.00000,1.00000,0.00000]

    d3_depth_camInfo.D = [0.126, -0.329, 0.111, -0.001, -0.002]
    d3_depth_camInfo.K = [388.198,0.00000,253.270,0.00000,389.033,213.934,0.00000,0.00000,1.00000]
    d3_depth_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d3_depth_camInfo.P = [388.198,0.00000,253.270,0.0,0.00000,389.033,213.934,0.0,0.00000,0.00000,1.00000,0.0]

    #CAMARA D4

    d4_color_camInfo = CameraInfo()
    d4_color_camInfo.header.frame_id = "d4_frame"
    d4_color_camInfo.height = 1080
    d4_color_camInfo.width = 1920
    d4_color_camInfo.distortion_model = "plumb_bob"

    d4_depth_camInfo = CameraInfo()
    d4_depth_camInfo.header.frame_id = "d4_frame"
    d4_depth_camInfo.height = 424
    d4_depth_camInfo.width = 512
    d4_depth_camInfo.distortion_model = "plumb_bob"


    d4_color_camInfo.D = [0.108, -0.125, 0.062, -0.001, -0.003]
    d4_color_camInfo.K = [1144.361,0.00000,966.359,0.00000,1147.337,548.038,0.00000,0.00000,1.00000]
    d4_color_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d4_color_camInfo.P = [1144.361,0.00000,966.359,0.0,0.00000,1147.337,548.038,0.0,0.00000,0.00000,1.00000,0.0]

    # d4_color_camInfo.K = [1066.08949,0.00000,961.62091,0.00000,1066.00068,535.42505,0.00000,0.00000,1.00000]
    # d4_color_camInfo.R = [0.99998,-0.00671,-0.00000,-0.00671,-0.99998,-0.00000,0.00000,-0.00000,1.00000]
    # d4_color_camInfo.P = [1066.06360,-7.42921,961.62091,56231.29035,-6.87077,-1065.97854,535.42505,-0.05257,0.00000,-0.00000,1.00000,0.00000]

    d4_depth_camInfo.D = [0.126, -0.329, 0.111, -0.001, -0.002]
    d4_depth_camInfo.K = [388.198,0.00000,253.270,0.00000,389.033,213.934,0.00000,0.00000,1.00000]
    d4_depth_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d4_depth_camInfo.P = [388.198,0.00000,253.270,0.0,0.00000,389.033,213.934,0.0,0.00000,0.00000,1.00000,0.0]

    #CAMARA D5

    d5_color_camInfo = CameraInfo()
    d5_color_camInfo.header.frame_id = "d5_frame"
    d5_color_camInfo.height = 1080
    d5_color_camInfo.width = 1920
    d5_color_camInfo.distortion_model = "plumb_bob"

    d5_depth_camInfo = CameraInfo()
    d5_depth_camInfo.header.frame_id = "d5_frame"
    d5_depth_camInfo.height = 424
    d5_depth_camInfo.width = 512
    d5_depth_camInfo.distortion_model = "plumb_bob"


    d5_color_camInfo.D = [0.108, -0.125, 0.062, -0.001, -0.003]
    d5_color_camInfo.K = [1144.361,0.00000,966.359,0.00000,1147.337,548.038,0.00000,0.00000,1.00000]
    d5_color_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d5_color_camInfo.P = [1144.361,0.00000,966.359,0.0,0.00000,1147.337,548.038,0.0,0.00000,0.00000,1.00000,0.0]

    # d5_color_camInfo.K = [1066.60026,0.00000,972.28097,0.00000,1066.56164,544.60657,0.00000,0.00000,1.00000]
    # d5_color_camInfo.R = [0.99999,0.00382,-0.00000,0.00382,-0.99999,-0.00000,0.00000,0.00000,1.00000]
    # d5_color_camInfo.P = [1066.59185,4.23662,972.28097,56231.37313,3.91713,-1066.55444,544.60657,-0.06764,0.00000,0.00000,1.00000,0.00000]

    d5_depth_camInfo.D = [0.126, -0.329, 0.111, -0.001, -0.002]
    d5_depth_camInfo.K = [388.198,0.00000,253.270,0.00000,389.033,213.934,0.00000,0.00000,1.00000]
    d5_depth_camInfo.R = [0.99997,0.00715,-0.00105,-0.00715,0.99995,0.00662,0.0011,-0.00661,0.99998]
    d5_depth_camInfo.P = [388.198,0.00000,253.270,0.0,0.00000,389.033,213.934,0.0,0.00000,0.00000,1.00000,0.0]

    r = rospy.Rate(5)

    while not rospy.is_shutdown():
        for d1_color, d1_depth, d3_color, d3_depth, d4_color, d4_depth, d5_color, d5_depth  in zip(sorted(os.listdir(d1_rooth+"/color/")),sorted(os.listdir(d1_rooth+"/depth/")),sorted(os.listdir(d3_rooth+"/color/")),sorted(os.listdir(d3_rooth+"/depth/")),sorted(os.listdir(d4_rooth+"/color/")),sorted(os.listdir(d4_rooth+"/depth/")),sorted(os.listdir(d5_rooth+"/color/")),sorted(os.listdir(d5_rooth+"/depth/"))):
            
            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D1_outputs/D1/color/")
            d1_img_col = cv2.imread(d1_color)
            d1_img_col = cv2.cvtColor(d1_img_col, cv2.COLOR_BGR2RGB)
            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D1_outputs/D1/depth/")
            d1_img_raw = cv2.imread(d1_depth, cv2.IMREAD_ANYDEPTH)

            d1_col_img_msg = bridge.cv2_to_imgmsg(d1_img_col, 'rgb8')
            d1_raw_img_msg = bridge.cv2_to_imgmsg(d1_img_raw)

            d1_col_img_msg.header.frame_id = "d1_frame"
            d1_raw_img_msg.header.frame_id = "d1_frame"


            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D3_outputs/D3/color/")
            d3_img_col = cv2.imread(d3_color)
            d3_img_col = cv2.cvtColor(d3_img_col, cv2.COLOR_BGR2RGB)
            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D3_outputs/D3/depth/")
            d3_img_raw = cv2.imread(d3_depth, cv2.IMREAD_ANYDEPTH)

            d3_col_img_msg = bridge.cv2_to_imgmsg(d3_img_col, 'rgb8')
            d3_raw_img_msg = bridge.cv2_to_imgmsg(d3_img_raw)

            d3_col_img_msg.header.frame_id = "d3_frame"
            d3_raw_img_msg.header.frame_id = "d3_frame"


            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D4_outputs/D4/color/")
            d4_img_col = cv2.imread(d4_color)
            d4_img_col = cv2.cvtColor(d4_img_col, cv2.COLOR_BGR2RGB)
            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D4_outputs/D4/depth/")
            d4_img_raw = cv2.imread(d4_depth, cv2.IMREAD_ANYDEPTH)

            d4_col_img_msg = bridge.cv2_to_imgmsg(d4_img_col, 'rgb8')
            d4_raw_img_msg = bridge.cv2_to_imgmsg(d4_img_raw)

            d4_col_img_msg.header.frame_id = "d4_frame"
            d4_raw_img_msg.header.frame_id = "d4_frame"


            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D5_outputs/D5/color/")
            d5_img_col = cv2.imread(d5_color)
            d5_img_col = cv2.cvtColor(d5_img_col, cv2.COLOR_BGR2RGB)
            os.chdir("/home/alejandro/Desktop/Gaelic_Football_Punt_Kick/D5_outputs/D5/depth/")
            d5_img_raw = cv2.imread(d5_depth, cv2.IMREAD_ANYDEPTH)

            d5_col_img_msg = bridge.cv2_to_imgmsg(d5_img_col, 'rgb8')
            d5_raw_img_msg = bridge.cv2_to_imgmsg(d5_img_raw)

            d5_col_img_msg.header.frame_id = "d5_frame"
            d5_raw_img_msg.header.frame_id = "d5_frame"


            d1_colimgPub.publish(d1_col_img_msg)
            d1_rawimgPub.publish(d1_raw_img_msg)

            d3_colimgPub.publish(d3_col_img_msg)
            d3_rawimgPub.publish(d3_raw_img_msg)

            d4_colimgPub.publish(d4_col_img_msg)
            d4_rawimgPub.publish(d4_raw_img_msg)

            d5_colimgPub.publish(d5_col_img_msg)
            d5_rawimgPub.publish(d5_raw_img_msg)


            d1_color_camInfoPub.publish(d1_color_camInfo)
            d1_depth_camInfoPub.publish(d1_depth_camInfo)

            d3_color_camInfoPub.publish(d3_color_camInfo)
            d3_depth_camInfoPub.publish(d3_depth_camInfo)

            d4_color_camInfoPub.publish(d4_color_camInfo)
            d4_depth_camInfoPub.publish(d4_depth_camInfo)

            d5_color_camInfoPub.publish(d5_color_camInfo)
            d5_depth_camInfoPub.publish(d5_depth_camInfo)
            
            r.sleep()



if __name__ == "__main__":
    rospy.init_node("global_node_base")
    main(sys.argv)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()