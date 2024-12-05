<!--
 * @Author: Msnakes 1327153842@qq.com
 * @Date: 2024-12-05 18:38:50
 * @LastEditors: Msnakes 1327153842@qq.com
 * @LastEditTime: 2024-12-05 19:33:40
 * @Description: 说明文档
 * 
-->
## 说明
<p>
yolo11的ros功能封装，主要有以下功能：<br>
1、服务的方式调用识别功能，并返回识别结果(bbox、mask、keypoint等)以及识别结果图像<br>
2、给定深度图像（和彩色图像配准后的），将返回识别出的物体的几何中心（2d平面几何中心或3d空间几何中心）对应的空间坐标<br>
3、给定目标坐标系，将物体的几何中心点坐标自动转换到该坐标系下<br>
</p>

## 依赖

### yolo11

<p>
根据网站安装：https://docs.ultralytics.com/zh/quickstart/#install-ultralytics
</p>

### open3d
``` pip install open3d ```

### ros noetic
### ros相关功能包
``` sudo apt install ros-noetic-tf2-geometry-msgs ros-noetic-cv-bridge ```

### 自定义消息类型说明
<p>
fnii_vision_detect/MultiStreamObjects:<br>
&ensp;&ensp;fnii_vision_detect/Objects[] results # 所有输入源的识别结果，目前最多只有一个，无需考虑多输入源的情况<br>
&ensp;&ensp;&ensp;&ensp;fnii_vision_detect/Object[] objects # 识别到的每个label的结果集合<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;int64[] mask  # 分割结果的掩膜，只有0、1，1表示该位置像素是该物体的一部分，需结合mask_height和mask_width转换为一般的图像格式( mask_height*mask_width 的多维数组)<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;uint32 mask_height # 掩膜图像的高<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;uint32 mask_width # 掩膜图像的宽<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;int64[] keypoints # 关键点，每两位表示一个关键点坐标(像素坐标)<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;int64[] bboxs # 识别框,[ux1, uy1, ux2, uy2]<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;string object_name # label名<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;float32 confidence # 置信度<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;geometry_msgs/Point[] centor_point # 对应几何中心点的空间坐标，目前数组长度最大为1<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;float64 x<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;float64 y<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;float64 z<br>
</p>

### 自定义服务类型说明
<p>
fnii_vision_detect/Detect：<br>
&ensp;&ensp;string rgb_topic # 彩色图像话题名
&ensp;&ensp;string target_class # 指定目标label，可不填，不填则返回所有识别到的label信息<br>
&ensp;&ensp;string depth_topic # 深度图像话题名（与彩色图像配准后的），可不填，不填则物体计算空间坐标点<br>
&ensp;&ensp;string depth_cam_info # 深度图像内参话题名（depth_topic填了，则此项必须填写）用于计算深度图像对应像素的空间坐标<br>
&ensp;&ensp;string frame_id # 填写了depth_topic才有效，转换目标的几何中心点到该坐标系下，<br>
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;可不填，不填则返回基于深度图像坐标系的几何中心点<br>
&ensp;&ensp;uint8 cal_centor_method # 几何中心点计算方式：0 识别框(bbox)的中心点，1 3维空间几何中心点， 2 关键点的中点<br>
&ensp;&ensp;---<br>
&ensp;&ensp;fnii_vision_detect/MultiStreamObjects results #检测的结果<br>
&ensp;&ensp;string err_msg # 如果检测失败，则有错误信息<br>
&ensp;&ensp;bool success # 检测的结果，true成功， false失败<br>
</p>

### 发布的话题
<p>
~detect_results_img(sensor_msgs/Image): 含识别结果的图片<br>
~detect_results(fnii_vision_detect/MultiStreamObjects): 话题形式的识别结果<br>
</p>

### 建立的服务
<p>
~detect(fnii_vision_detect/Detect):调用该服务进行识别<br>
</p>

### 参数说明
<p>
~weight：权重文件名<br>
</p>
