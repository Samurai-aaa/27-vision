string number
string type
float32 distance_to_image_center
geometry_msgs/Pose pose
# NN 检测角点的重投影误差（像素），tracker 门控用
float64 reproj_err
# NN 检测的 4 角点原图像素坐标（顺序 TL,BL,BR,TR，x,y 交错 0..7）。
# 有观测帧 tracker 画"贴合实测框"，掉帧时无此字段（不发布该板）。
float32[8] corners_px
# Object score and raw top-two class-logit margin (not a calibrated probability).
float32 confidence
float32 class_margin
bool color_uncertain
