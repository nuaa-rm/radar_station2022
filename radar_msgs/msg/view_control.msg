bool cameraFullScreen   # 是否将相机显示放置全屏（小地图等功能将被隐藏）
string camera           # 当前显示的camera的名称（同配置文件），空字符串将显示no camera，由前端选择
float32 scale           # 缩放比例，必须大于等于1
float32 x               # 原图比例位置X（坐标/边长），该位置将被作为缩放后显示的中心
float32 y               # 续上，即[0.5, 0.5]位置为以原图中心为缩放中心缩放
