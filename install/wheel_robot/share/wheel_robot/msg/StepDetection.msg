std_msgs/Header header

bool detected
float32 distance
float32 height
float32 confidence

# 台阶前沿在相机地面坐标系中的姿态。
# edge_angle_rad=0 表示台阶横边与相机 x 轴平行，即车头正对台阶。
# 正值表示拟合直线 z=a*x+b 的 a>0；lateral_offset_m 正值表示台阶中心在车体右侧。
bool pose_valid
float32 edge_angle_rad
float32 lateral_offset_m

uint32 confirm_frames
uint32 required_confirm_frames
