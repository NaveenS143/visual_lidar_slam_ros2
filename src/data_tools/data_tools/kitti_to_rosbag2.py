import sys

try:
    import pykitti
except ImportError as e:
    print("Please install pykitti with `pip install pykitti`")
    sys.exit(1)

from rclpy.time import Time
from datetime import datetime
from sensor_msgs_py import point_cloud2 as pcl2
import tf2_ros
from tf2_msgs.msg import TFMessage
import os
import rosbag2_py
import progressbar
from sensor_msgs.msg import CameraInfo, NavSatFix, Imu, PointField, CompressedImage, PointCloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
import numpy as np
import argparse
import cv2
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
from rclpy.serialization import serialize_message
from rosbag2_py import TopicMetadata
import yaml
from pyproj import Proj, Transformer

def save_dynamic_tf(bag, kitti, kitti_type, initial_time):
    print("Exporting time dependent transformations (UTM coordinates)")
    topic_info = TopicMetadata(
        name="/tf",
        type="tf2_msgs/msg/TFMessage",
        serialization_format="cdr",
    )
    bag.create_topic(topic_info)

    # Initialize UTM transformer
    # Automatically determine UTM zone from first latitude/longitude
    if kitti.oxts and len(kitti.oxts) > 0:
        lat, lon = kitti.oxts[0].packet.lat, kitti.oxts[0].packet.lon
        utm_zone = int((lon + 180) // 6) + 1  # Calculate UTM zone
        utm_proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84', south=lat < 0)
        transformer = Transformer.from_crs("epsg:4326", utm_proj.crs, always_xy=True)
    else:
        raise ValueError("No OXTS data available for UTM conversion")

    if kitti_type.find("raw") != -1:
        for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
            tf_msg = TFMessage()
            tf_transform = TransformStamped()

            seconds = int(timestamp.timestamp())
            nanoseconds = int((timestamp.timestamp() - seconds) * 1e9)
            tf_transform.header.stamp = Time(
                seconds=seconds, nanoseconds=nanoseconds
            ).to_msg()

            # World to base_link transform (vehicle pose in UTM)
            tf_transform.header.frame_id = "world"
            tf_transform.child_frame_id = "base_link"

            # Convert lat/lon to UTM
            easting, northing = transformer.transform(oxts.packet.lon, oxts.packet.lat)
            altitude = oxts.packet.alt

            # Use UTM coordinates for translation
            tf_transform.transform.translation.x = easting
            tf_transform.transform.translation.y = northing
            tf_transform.transform.translation.z = altitude

            # Orientation (use the same quaternion from roll, pitch, yaw)
            q = quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
            tf_transform.transform.rotation.x = float(q[0])
            tf_transform.transform.rotation.y = float(q[1])
            tf_transform.transform.rotation.z = float(q[2])
            tf_transform.transform.rotation.w = float(q[3])

            tf_msg.transforms.append(tf_transform)

            bag.write(
                "/tf",
                serialize_message(tf_msg),
                int(tf_transform.header.stamp.sec * 1e9 + tf_transform.header.stamp.nanosec),
            )

    elif kitti_type.find("odom") != -1:
        # For odometry dataset, you may need to adjust based on available data
        # If ground truth poses are in a local coordinate system, UTM conversion may not apply directly
        print("Warning: UTM conversion for odometry dataset may require additional handling.")
        for i, (timestamp, tf_matrix) in enumerate(zip(kitti.timestamps, kitti.T_w_cam0)):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()

            dt = initial_time + timestamp.total_seconds()
            seconds = int(dt)
            nanoseconds = int((dt - seconds) * 1e9)
            tf_stamped.header.stamp = Time(
                seconds=seconds, nanoseconds=nanoseconds
            ).to_msg()

            tf_stamped.header.frame_id = "world"
            tf_stamped.child_frame_id = "camera_left"

            # For odometry, use the existing transform matrix or convert if lat/lon available
            t = tf_matrix[0:3, 3]
            q = quaternion_from_matrix(tf_matrix)

            tf_stamped.transform.translation.x = float(t[0])
            tf_stamped.transform.translation.y = float(t[1])
            tf_stamped.transform.translation.z = float(t[2])

            tf_stamped.transform.rotation.x = float(q[0])
            tf_stamped.transform.rotation.y = float(q[1])
            tf_stamped.transform.rotation.z = float(q[2])
            tf_stamped.transform.rotation.w = float(q[3])

            tf_msg.transforms.append(tf_stamped)

            bag.write(
                "/tf",
                serialize_message(tf_msg),
                int(tf_stamped.header.stamp.sec * 1e9 + tf_stamped.header.stamp.nanosec),
            )


def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    topic_info = TopicMetadata(
        name=topic,
        type="sensor_msgs/msg/Imu",
        serialization_format="cdr",
    )

    bag.create_topic(topic_info)
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        # Correct quaternion order: w, x, y, z (Hamilton convention)
        q = quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        
        # Convert timestamp properly
        seconds = int(timestamp.timestamp())
        nanoseconds = int((timestamp.timestamp() - seconds) * 1e9)
        imu.header.stamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        
        # Quaternion in ROS: x, y, z, w
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        
        # Linear acceleration (m/s^2)
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        
        # Angular velocity (rad/s)
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        
        # Set covariance matrices (use -1 for unknown)
        imu.orientation_covariance = [-1.0] * 9
        imu.angular_velocity_covariance = [-1.0] * 9
        imu.linear_acceleration_covariance = [-1.0] * 9
        
        bag.write(
            topic,
            serialize_message(imu),
            int(imu.header.stamp.sec * 1e9 + imu.header.stamp.nanosec),
        )


def save_camera_data(
    bag, kitti_type, kitti, util, bridge, camera, camera_frame_id, topic, initial_time
):
    print("Exporting camera {}".format(camera))
    
    if kitti_type.find("raw") != -1:
        camera_pad = "{0:02d}".format(camera)
        image_dir = os.path.join(kitti.data_path, "image_{}".format(camera_pad))
        image_path = os.path.join(image_dir, "data")
        image_filenames = sorted(os.listdir(image_path))
        
        # Use KITTI timestamps directly
        image_datetimes = []
        for timestamp in kitti.timestamps:
            seconds = int(timestamp.timestamp())
            nanoseconds = int((timestamp.timestamp() - seconds) * 1e9)
            image_datetimes.append(
                Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
            )

        # Camera calibration
        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.width, calib.height = tuple(
            [int(x) for x in util["S_rect_{}".format(camera_pad)].tolist()]
        )
        calib.distortion_model = "plumb_bob"
        calib.k = util["K_{}".format(camera_pad)].flatten().tolist()
        calib.r = util["R_rect_{}".format(camera_pad)].flatten().tolist()
        calib.d = util["D_{}".format(camera_pad)].flatten().tolist()
        calib.p = util["P_rect_{}".format(camera_pad)].flatten().tolist()

    elif kitti_type.find("odom") != -1:
        camera_pad = "{0:01d}".format(camera)
        image_path = os.path.join(kitti.sequence_path, "image_{}".format(camera_pad))
        image_filenames = sorted(os.listdir(image_path))
        
        image_datetimes = []
        for timestamp in kitti.timestamps:
            dt = initial_time + timestamp.total_seconds()
            seconds = int(dt)
            nanoseconds = int((dt - seconds) * 1e9)
            image_datetimes.append(Time(seconds=seconds, nanoseconds=nanoseconds).to_msg())

        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.p = util["P{}".format(camera_pad)].flatten().tolist()

    # Create topics
    topic_info = TopicMetadata(
        name=topic + "/image_raw/compressed",
        type="sensor_msgs/msg/CompressedImage",
        serialization_format="cdr",
    )
    bag.create_topic(topic_info)
    
    topic_info1 = TopicMetadata(
        name=topic + "/camera_info",
        type="sensor_msgs/msg/CameraInfo",
        serialization_format="cdr",
    )
    bag.create_topic(topic_info1)

    iterable = zip(image_datetimes, image_filenames)
    bar = progressbar.ProgressBar()
    
    for dt, filename in bar(iterable):
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)
        
        if cv_image is None:
            continue
            
        # Update calibration with actual image dimensions
        calib.height, calib.width = cv_image.shape[:2]
        
        # Convert grayscale cameras
        if camera in (0, 1):
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Create compressed image message
        image_message = CompressedImage()
        image_message.format = "png"
        image_message.data = np.array(cv2.imencode(".png", cv_image)[1]).tobytes()
        image_message.header.frame_id = camera_frame_id
        image_message.header.stamp = dt
        
        calib.header.stamp = dt

        bag.write(
            topic + "/image_raw/compressed",
            serialize_message(image_message),
            int(image_message.header.stamp.sec * 1e9 + image_message.header.stamp.nanosec),
        )
        bag.write(
            topic + "/camera_info",
            serialize_message(calib),
            int(calib.header.stamp.sec * 1e9 + calib.header.stamp.nanosec),
        )


def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    topic_info = TopicMetadata(
        name=topic + "/pointcloud",
        type="sensor_msgs/msg/PointCloud2",
        serialization_format="cdr",
    )
    bag.create_topic(topic_info)
    
    velo_path = os.path.join(kitti.data_path, "velodyne_points")
    velo_data_dir = os.path.join(velo_path, "data")
    velo_filenames = sorted(os.listdir(velo_data_dir))
    
    velo_time_stamps = []
    for timestamp in kitti.timestamps:
        seconds = int(timestamp.timestamp())
        nanoseconds = int((timestamp.timestamp() - seconds) * 1e9)
        velo_time_stamps.append(Time(seconds=seconds, nanoseconds=nanoseconds).to_msg())

    iterable = zip(velo_time_stamps, velo_filenames)
    bar = progressbar.ProgressBar()
    
    for dt, filename in bar(iterable):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # Read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # Create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = dt

        # Define point fields
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create point cloud message
        pcl_msg = pcl2.create_cloud(header, fields, scan)

        bag.write(
            topic + "/pointcloud",
            serialize_message(pcl_msg),
            int(pcl_msg.header.stamp.sec * 1e9 + pcl_msg.header.stamp.nanosec),
        )


def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg


def inv(transform):
    """Invert rigid body transformation matrix"""
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_static_transforms(bag, transforms, timestamps):
    print("Exporting static transformations")
    
    # Create topic with proper QoS
    qos_profile = {
        "history": 1,  # KEEP_LAST
        "depth": 1,
        "reliability": 1,  # RELIABLE
        "durability": 1,   # TRANSIENT_LOCAL
        "deadline": {"sec": 0, "nsec": 0},
        "lifespan": {"sec": 0, "nsec": 0},
        "liveliness": 1,
        "liveliness_lease_duration": {"sec": 0, "nsec": 0},
        "avoid_ros_namespace_conventions": False,
    }
    
    qos_profile_str = yaml.dump(qos_profile, default_flow_style=False)
    qos_profile_str = "- " + qos_profile_str.replace('\n', '\n  ').strip()
    
    topic_info = TopicMetadata(
        name="/tf_static",
        type="tf2_msgs/msg/TFMessage",
        serialization_format="cdr",
        offered_qos_profiles=qos_profile_str,
    )

    bag.create_topic(topic_info)
    
    # Create TF message with all static transforms
    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(
            from_frame_id=transform[0], 
            to_frame_id=transform[1], 
            transform=transform[2]
        )
        tfm.transforms.append(t)
    
    # Publish static transforms with first timestamp
    if timestamps:
        first_timestamp = timestamps[0]
        seconds = int(first_timestamp.timestamp())
        nanoseconds = int((first_timestamp.timestamp() - seconds) * 1e9)
        time = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        
        for transform in tfm.transforms:
            transform.header.stamp = time
            
        bag.write(
            "/tf_static", 
            serialize_message(tfm), 
            int(time.sec * 1e9 + time.nanosec)
        )


def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    print("Exporting GPS fix data")
    topic_info = TopicMetadata(
        name=topic,
        type="sensor_msgs/msg/NavSatFix",
        serialization_format="cdr",
    )
    bag.create_topic(topic_info)
    
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        
        seconds = int(timestamp.timestamp())
        nanoseconds = int((timestamp.timestamp() - seconds) * 1e9)
        navsatfix_msg.header.stamp = Time(
            seconds=seconds, nanoseconds=nanoseconds
        ).to_msg()
        
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1  # GPS
        navsatfix_msg.status.status = 0   # STATUS_FIX
        
        # Set covariance (diagonal matrix for simplicity)
        navsatfix_msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        navsatfix_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]        
        bag.write(
            topic,
            serialize_message(navsatfix_msg),
            int(navsatfix_msg.header.stamp.sec * 1e9 + navsatfix_msg.header.stamp.nanosec),
        )


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    print("Exporting GPS velocity data")
    topic_info = TopicMetadata(
        name=topic,
        type="geometry_msgs/msg/TwistStamped",
        serialization_format="cdr",
    )
    bag.create_topic(topic_info)
    
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        
        seconds = int(timestamp.timestamp())
        nanoseconds = int((timestamp.timestamp() - seconds) * 1e9)
        twist_msg.header.stamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        
        # Linear velocity
        twist_msg.twist.linear.x = oxts.packet.vf  # forward
        twist_msg.twist.linear.y = oxts.packet.vl  # left
        twist_msg.twist.linear.z = oxts.packet.vu  # up
        
        # Angular velocity
        twist_msg.twist.angular.x = oxts.packet.wf  # roll rate
        twist_msg.twist.angular.y = oxts.packet.wl  # pitch rate
        twist_msg.twist.angular.z = oxts.packet.wu  # yaw rate

        bag.write(
            topic,
            serialize_message(twist_msg),
            int(twist_msg.header.stamp.sec * 1e9 + twist_msg.header.stamp.nanosec),
        )


def main():
    parser = argparse.ArgumentParser(
        description="Convert KITTI dataset to ROS2 bag file the easy way!"
    )
    
    # Accepted argument values
    kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    odometry_sequences = [str(s).zfill(2) for s in range(22)]

    parser.add_argument("kitti_type", choices=kitti_types, help="KITTI dataset type")
    parser.add_argument(
        "dir",
        nargs="?",
        default=os.getcwd(),
        help="base directory of the dataset, if no directory passed the default is current working directory",
    )
    parser.add_argument(
        "-t",
        "--date",
        help="date of the raw dataset (i.e. 2011_09_26), option is only for RAW datasets.",
    )
    parser.add_argument(
        "-r",
        "--drive",
        help="drive number of the raw dataset (i.e. 0001), option is only for RAW datasets.",
    )
    parser.add_argument(
        "-s",
        "--sequence",
        choices=odometry_sequences,
        help="sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.",
    )
    args = parser.parse_args()

    bridge = CvBridge()

    # CAMERAS
    cameras = [
        (0, "camera_gray_left", "/kitti/camera_gray_left"),
        (1, "camera_gray_right", "/kitti/camera_gray_right"),
        (2, "camera_color_left", "/kitti/camera_color_left"),
        (3, "camera_color_right", "/kitti/camera_color_right"),
    ]

    # Create bag
    bag = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(
        uri="datasets/kitti_bag_00", storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions("", "")
    bag.open(storage_options, converter_options)

    try:
        if args.kitti_type.find("raw") != -1:
            # Validate arguments
            if args.date is None:
                print("Date option is not given. It is mandatory for raw dataset.")
                print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
                sys.exit(1)
            elif args.drive is None:
                print("Drive option is not given. It is mandatory for raw dataset.")
                print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
                sys.exit(1)
                
            # Load KITTI data
            kitti = pykitti.raw(args.dir, args.date, args.drive)
            if not os.path.exists(kitti.data_path):
                print("Path {} does not exist. Exiting.".format(kitti.data_path))
                sys.exit(1)

            if len(kitti.timestamps) == 0:
                print("Dataset is empty? Exiting.")
                sys.exit(1)

            # Frame IDs
            imu_frame_id = "imu_link"
            velo_frame_id = "velo_link"
            
            # Topics
            imu_topic = "/kitti/oxts/imu"
            gps_fix_topic = "/kitti/oxts/gps/fix"
            gps_vel_topic = "/kitti/oxts/gps/vel"
            velo_topic = "/kitti/velo"

            # Static transforms based on KITTI calibration
            T_base_link_to_imu = np.eye(4)
            T_base_link_to_imu[0:3, 3] = [-2.71/2.0 - 0.05, 0.32, 0.93]  # Approximate vehicle to IMU

            transforms = [
                ("base_link", imu_frame_id, T_base_link_to_imu),
                (imu_frame_id, velo_frame_id, inv(kitti.calib.T_velo_imu)),
                (imu_frame_id, cameras[0][1], inv(kitti.calib.T_cam0_imu)),
                (imu_frame_id, cameras[1][1], inv(kitti.calib.T_cam1_imu)),
                (imu_frame_id, cameras[2][1], inv(kitti.calib.T_cam2_imu)),
                (imu_frame_id, cameras[3][1], inv(kitti.calib.T_cam3_imu)),
            ]

            # Load camera calibration
            util = pykitti.utils.read_calib_file(
                os.path.join(kitti.calib_path, "calib_cam_to_cam.txt")
            )

            # Export data
            save_static_transforms(bag, transforms, kitti.timestamps)
            save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=None)
            save_imu_data(bag, kitti, imu_frame_id, imu_topic)
            save_gps_fix_data(bag, kitti, imu_frame_id, gps_fix_topic)
            save_gps_vel_data(bag, kitti, imu_frame_id, gps_vel_topic)
            
            for camera in cameras:
                save_camera_data(
                    bag,
                    args.kitti_type,
                    kitti,
                    util,
                    bridge,
                    camera=camera[0],
                    camera_frame_id=camera[1],
                    topic=camera[2],
                    initial_time=None,
                )
                
            save_velo_data(bag, kitti, velo_frame_id, velo_topic)

        elif args.kitti_type.find("odom") != -1:
            if args.sequence is None:
                print("Sequence option is not given. It is mandatory for odometry dataset.")
                print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
                sys.exit(1)

            kitti = pykitti.odometry(args.dir, args.sequence)
            if not os.path.exists(kitti.sequence_path):
                print("Path {} does not exist. Exiting.".format(kitti.sequence_path))
                sys.exit(1)

            kitti.load_calib()
            kitti.load_timestamps()

            if len(kitti.timestamps) == 0:
                print("Dataset is empty? Exiting.")
                sys.exit(1)

            if args.sequence in odometry_sequences[:11]:
                print("Odometry dataset sequence {} has ground truth information (poses).".format(args.sequence))
                kitti.load_poses()

            util = pykitti.utils.read_calib_file(
                os.path.join(args.dir, "sequences", args.sequence, "calib.txt")
            )
            
            current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
            
            # Select cameras based on type
            if args.kitti_type.find("gray") != -1:
                used_cameras = cameras[:2]
            elif args.kitti_type.find("color") != -1:
                used_cameras = cameras[-2:]

            save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=current_epoch)
            
            for camera in used_cameras:
                save_camera_data(
                    bag,
                    args.kitti_type,
                    kitti,
                    util,
                    bridge,
                    camera=camera[0],
                    camera_frame_id=camera[1],
                    topic=camera[2],
                    initial_time=current_epoch,
                )

    finally:
        print("## OVERVIEW ##")
        print("Bag conversion completed successfully!")
        bag.close()


if __name__ == "__main__":
    main()

# Usage: ros2 run data_tools kitti_to_rosbag2 raw_synced /home/naveen/slam_ws/datasets/kitti/2011_10_03_drive_0027_sync -t 2011_10_03 -r 0027
