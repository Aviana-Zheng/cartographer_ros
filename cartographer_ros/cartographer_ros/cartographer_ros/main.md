cartographer_ros说明

```shell
1. cartographer_node
   |===@订阅的Topic
   |---------scan (sensors_msgs/LaserScan):   解释：传感器数据
   |---------echoes (sensors_msgs/MultiEchoLaserScan) 解释：没太看明白这个是做什么用，感觉可能是设置如果有多个Laser传感器的情况
   |---------points2 (sensors_msgs/PointCloud2) 解释：2D点云数据
   |---------imu (sensors_msgs/Imu) 解释：IMU的数据
   |---------odom (nav_msgs/Odometry) 解释：里程计数据
   |===@发布的Topic
   |---------scan_matched_points2 (sensors_msgs/PointCloud2) 解释：匹配好的2D点云数据，用来scan-to-submap matching
   |---------submap_list (cartographer_ros_msgs/SubmapList) 解释：发布构建好的submap。
   |===@提供的Service
   |---------submap_query (cartographer_ros_msgs/SubmapQuery) 解释：可以提供查询submap的服务，获取到request的submap
   |---------start_trajectory (cartographer_ros_msgs/StartTrajectory) 解释：维护一条轨迹
   |---------finish_trajectory (cartographer_ros_msgs/FinishTrajectory) 解释：Finish一个给定ID的轨迹
   |---------write_state (cartographer_ros_msgs/WriteState) 解释：将当前状态写入磁盘文件中
   |---------get_trajectory_states (cartographer_ros_msgs/GetTrajectoryStates) 解释：获取指定trajectory的状态
   |---------read_metrics (cartographer_ros_msgs/ReadMetrics) 
   |===@Required tf Transforms
   |===@Provided tf Transforms
2. offline_node
   |===可以理解为一个快速版本的cartographer
   |===不监听任何topic,二是直接从数据包中读取传感器数据。发布的Topic与cartographer_node相同，除此以外，还有：
   |===@额外发布的Topic
   |---------~bagfile_progress (cartographer_ros_msgs/BagfileProgress) 解释：可查询处理包的进度等情况
   |===@Parameters
   |---------~bagfile_progress_pub_interval(double, default=10.0) 解释：发布包数据的时间间隔。以s为单位；
3. occupancy_grid_node
   |===description：主要任务是监听submap_list这个Topic然后构建栅格地图并发布
   |===@订阅的Topics
   |---------submap_list (cartographer_ros_msgs/SubmapList) 解释： 由cartographer_node发布
   |===@发布的Topics
   |---------map (nav_msgs/OccupancyGrid) 解释：栅格地图
4. pbstream_map_publisher
   |===description: 快速版的occupancy_grid_node
   |===未订阅任何节点
   |===发布的Topics
   |---------map (nav_msgs/OccupancyGrid) 解释：栅格地图
```



