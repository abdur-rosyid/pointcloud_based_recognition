# pointcloud_based_recognition
ROS packages for object detection and pose estimation based on 3D pointcloud

- lidar_node: a ROS package to detect and estimate the pose of an object based on 3D pointcloud. The 3D pointcloud should be acquired by using sensors such as 3D Lidar. The detection and pose estimation are performed by using some basic PCL functions. The estimated pose consists of both the position and the orientation of the object. This estimated pose is represented by a ROS frame attached to the object. The 3D position of the object is defined by a point (x,y,z) representing the centroid of the object. Notice that this centroid is located on the surface of the detected object. The point (x,y,z) is also the origin of the created object frame. This position is expressed with respect to the sensor frame. The orientation of the object is represented by the orientation of the created object frame, where one of the frame axis is aligned with the surface of the object, i.e. the orientation of the object surface about the vertical axis. 
