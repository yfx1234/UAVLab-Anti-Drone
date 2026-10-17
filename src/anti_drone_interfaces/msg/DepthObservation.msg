# 模拟双目与真实双目感知节点共用的 V1 稳定接口。
# header.stamp 必须是图像曝光/采集时刻的 ROS 时间，不是推理完成或发布时间。
# header.frame_id 必须是标准 optical frame：+X 向右、+Y 向下、+Z 沿光轴。
uint16 INTERFACE_VERSION_V1=1
uint16 interface_version
std_msgs/Header header
uint32 sequence
uint32 track_id
bool visible
float32 confidence
uint32 image_width
uint32 image_height
float64 pixel_u
float64 pixel_v
bool bounding_box_valid
float64 bounding_box_width_px
float64 bounding_box_height_px
bool angles_valid
float64 horizontal_angle_rad
float64 vertical_angle_rad
float64[4] angle_covariance
bool depth_valid
float64 depth_z_m
float64 range_m
geometry_msgs/Point position_optical
float64[9] position_covariance
