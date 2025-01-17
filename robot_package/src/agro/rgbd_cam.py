#!/usr/bin/python3

import pyrealsense2.pyrealsense2 as rs
import time
import ros_numpy
import rospy
import numpy as np
import cv2
from agro.msg import IMU, Gyro, Accel, Intrinsic, Extrinsic, CamParam
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

class RGBD:
    def __init__(self, device_info = [], width=640, height=480):
        self.device_serial, self.name, self.accel_fps, self.gyro_fps = device_info
        self.gyro = Gyro()
        self.accel = Accel()
        self.imu = IMU()

        self.width = width
        self.height = height
        self.bridge = CvBridge()

        self.frames = None
        self.color_frame = None
        self.depth_frame = None
        self.depth_intrinsics = Intrinsic()
        self.color_intrinsics = Intrinsic()
        self.depth_to_color_extrinsic = Extrinsic()
        self.cam_param = CamParam()

        self.depth_colormap = None
        self.pc = rs.pointcloud()
        self.decimate = rs.decimation_filter()
        self.points = None
        self.point_cloud_arr = []

        self.rs_color_intrinsics = None
        self.rs_depth_intrinsics = None
        self.rs_depth_to_color_extrinsic = None
        self.depth_scale = None

        self.cam_intrinsic = {"width":None, "height":None, "ppx":None\
                            ,"ppy":None, "fx":None, "fy":None, "model":None, "coeffs": None}

        self.depth_pixel = [200, 200]

        self.pipeline = rs.pipeline()
        self.align = None
        self.align_to = None
        self.raw_depth = None
        self.raw_color = None

        self.rgbd_color_pub = rospy.Publisher('rgbd_color_'+self.name, Image, queue_size=10)
        self.rgbd_depth_pub = rospy.Publisher('rgbd_depth_'+self.name, Image, queue_size=10)
        self.rgbd_pc_pub = rospy.Publisher('rgbd_point_cloud_'+self.name, PointCloud2, queue_size=10)
        self.cam_param_pub = rospy.Publisher('rgbd_param_'+self.name, CamParam, queue_size=10)  
        
        self.initializeCamera()  
        time.sleep(2)


    def initializeCamera(self):
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        config = rs.config()
        config.enable_device(self.device_serial)
        # config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, self.accel_fps)
        # config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.gyro_fps)

        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16 , 30)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
        pipeline_profile = config.resolve(pipeline_wrapper)
        if config.can_resolve(pipeline_wrapper):
            print("RGBD_{} with id: {} is connected successfully!".format(self.name, self.device_serial))
            self.pipeline.start(config)
        
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        
    def setGyroData(self, gyro):
        self.gyro.x = gyro.x
        self.gyro.y = gyro.y
        self.gyro.z = gyro.z

    def setAccelData(self, accel):
        self.accel.x = accel.x
        self.accel.y = accel.y
        self.accel.z = accel.z

    def getFrames(self):
        # self.frames = self.align.process(self.pipeline.wait_for_frames())
        self.frames = self.pipeline.wait_for_frames()
        self.getRawData()
        self.numpifyFrames()

    def getRawData(self):
        self.raw_depth = self.frames.get_depth_frame()
        self.raw_color = self.frames.get_color_frame()

    def numpifyFrames(self):
        self.depth_frame = np.asanyarray(self.raw_depth.get_data())
        self.color_frame = np.asanyarray(self.raw_color.get_data())

    def getCamParameters(self):
        self.rs_color_intrinsics = self.raw_color.profile.as_video_stream_profile().get_intrinsics()
        self.rs_depth_intrinsics = self.raw_depth.profile.as_video_stream_profile().get_intrinsics()
        self.rs_depth_to_color_extrinsic = self.raw_depth.profile.get_extrinsics_to(self.raw_color.profile)
        self.getIMU()
        self.getDepthIntrinsic()
        self.getColorIntrinsic()
        self.getDepthToColorExtrinsic()
        self.cam_param_pub.publish(self.cam_param)

    def getIMU(self):
        self.setAccelData(self.frames[2].as_motion_frame().get_motion_data())
        self.setGyroData(self.frames[3].as_motion_frame().get_motion_data())
        self.imu.accel = self.accel
        self.imu.gyro = self.gyro
        self.cam_param.imu = self.imu

    def getDepthColorMap(self):
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_frame, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = self.depth_colormap.shape
        color_colormap_dim = self.color_frame.shape

        # # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            self.color_frame = cv2.resize(self.color_frame, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

    def getDepthIntrinsic(self):
        self.depth_intrinsics.width = self.rs_depth_intrinsics.width
        self.depth_intrinsics.height = self.rs_depth_intrinsics.height
        self.depth_intrinsics.ppx = self.rs_depth_intrinsics.ppx
        self.depth_intrinsics.ppy = self.rs_depth_intrinsics.ppy
        self.depth_intrinsics.fx = self.rs_depth_intrinsics.fx
        self.depth_intrinsics.fy = self.rs_depth_intrinsics.fy
        self.depth_intrinsics.model = self.rs_depth_intrinsics.model
        self.depth_intrinsics.coeffs = self.rs_depth_intrinsics.coeffs
        self.cam_param.depth_intrinsic = self.depth_intrinsics


    def getColorIntrinsic(self):
        self.color_intrinsics.width = self.rs_color_intrinsics.width
        self.color_intrinsics.height = self.rs_color_intrinsics.height
        self.color_intrinsics.ppx = self.rs_color_intrinsics.ppx
        self.color_intrinsics.ppy = self.rs_color_intrinsics.ppy
        self.color_intrinsics.fx = self.rs_color_intrinsics.fx
        self.color_intrinsics.fy = self.rs_color_intrinsics.fy
        self.color_intrinsics.model = self.rs_color_intrinsics.model
        self.color_intrinsics.coeffs = self.rs_color_intrinsics.coeffs
        self.cam_param.color_intrinsic = self.color_intrinsics

    def getDepthToColorExtrinsic(self):
        self.depth_to_color_extrinsic.rotation = self.rs_depth_to_color_extrinsic.rotation
        self.depth_to_color_extrinsic.translation = self.rs_depth_to_color_extrinsic.translation
        self.cam_param.extrinsic = self.depth_to_color_extrinsic

    
    def getPointCloud(self):
        
        self.points = self.pc.calculate(self.raw_depth)
        vertices = np.asarray((self.points.get_vertices())).tolist()
        # print(vertices)

        # nonzero_vertices = np.asarray(vertices[np.nonzero(vertices)])
        # tex_coords = np.asarray(self.points.get_texture_coordinates())
        # nz = np.array([list(vertex) for vertex in nonzero_vertices])
        # pc_array = np.zeros(len(nz), dtype=[('x', np.float32),('y', np.float32),('z', np.float32)])
        # pc_array['x'] = nz[:,0]
        # pc_array['y'] = nz[:,1]
        # pc_array['z'] = nz[:,2]
        # self.point_cloud_arr = ros_numpy.msgify(PointCloud2, pc_array)
        # self.rgbd_pc_pub.publish(self.point_cloud_arr)
        #########################################################
        # # get depth resolution:
        height, width = self.depth_frame.shape
        length = height * width
        # compute indices:
        jj = np.tile(range(width), height)
        ii = np.repeat(range(height), width)
        # rechape depth image
        z = self.depth_frame.reshape(length)
        # compute pcd:
        pcd = np.dstack([(ii - self.depth_intrinsics.ppx) * z / self.depth_intrinsics.fx,
                        (jj - self.depth_intrinsics.ppy) * z / self.depth_intrinsics.fy,
                        z]).reshape((length, 3))

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

        # header = Header()
        # header.frame_id = "map"
        # header.stamp = rospy.Time.now()
        # bigendian = False


        # pc2 = point_cloud2.create_cloud(header, fields, pcd)

        # # self.point_cloud_arr = ros_numpy.msgify(PointCloud2, pcd)
        # self.rgbd_pc_pub.publish(self.point_cloud_arr)


    def getDepth(self):
        self.getDepthColorMap()
        self.rgbd_depth_pub.publish(self.bridge.cv2_to_imgmsg(self.depth_colormap, "bgr8"))
        # self.rgbd_depth_pub.publish(self.bridge.cv2_to_imgmsg(self.depth_frame, "mono16"))
    
    def getRGB(self):
        self.rgbd_color_pub.publish(self.bridge.cv2_to_imgmsg(self.color_frame, "bgr8"))

    def toPLY(self):
        colorizer = rs.colorizer()
        colorized = colorizer.process(self.frames)
        ply = rs.save_to_ply("/home/aida/Documents/{}.ply".format(self.name))
        ply.set_option(rs.save_to_ply.option_ply_binary, False)
        ply.set_option(rs.save_to_ply.option_ply_normals, True)
        ply.process(colorized)

    def operate(self):
        try:
            while not rospy.is_shutdown():
                t_get_frame = time.time()
                self.getFrames()
                diff_get_frame = time.time() - t_get_frame
                # print(self.name+" get frame time: ", diff_get_frame)

                t_get_cam_param = time.time()
                # self.getCamParameters()
                diff_get_cam_param = time.time() - t_get_cam_param
                # print(self.name+" get cam param time: ", diff_get_cam_param)
                # self.getDepthColorMap()

                # depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, self.depth_pixel, self.depth_scale)
                # color_point = rs.rs2_transform_point_to_point(self.depth_to_color_extrinsic, depth_point)
                
                t_get_rgb = time.time()
                self.getRGB()
                diff_get_rgb = time.time() - t_get_rgb
                # print(self.name+" get rgb time: ", diff_get_rgb)

                t_get_depth = time.time()
                self.getDepth()
                diff_get_depth = time.time() - t_get_depth
                # print(self.name+" get depth time: ", diff_get_depth)

                t_pc = time.time()
                # self.getPointCloud()
                diff_pc = time.time() - t_pc
                # print(self.name+" get pointcloud time: ", diff_pc)

                t_total = diff_get_frame + diff_get_cam_param + diff_get_rgb + diff_get_depth + diff_pc

                # print(self.name+"_total_time: ", t_total)

                rospy.Rate(30).sleep()
            #     cv2.imshow(self.name, self.color_frame)
            #     if cv2.waitKey(1) & 0xFF == ord('q'): break
            # cv2.destroyAllWindows()
            rospy.spin()
        finally:
            self.pipeline.stop()