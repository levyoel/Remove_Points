# Remove_Points

#### 介绍
手动去除地图动态点云

#### 软件架构
软件架构说明


#### 安装教程

需要安装PCL库和nlohmann库

安装PCL库
sudo apt-get install libpcl-dev

安装nlohmann库
下载git clone https://github.com/nlohmann/json.git
cd json
mkdir build
cmake ..
make
sudo make install

#### 使用说明
将需要剔除动态点的"input_pcd_file".pcd放在build目录下
"input_pcd_file"改为config.json中对应参数的值
项目启动
cd build
cmake ..
make
./remove_dynamic_points

调整点云地图比例，选择需要剔除的区域
按下s，终端提示Selection started.
按shift+鼠标左键选择框选区域顶点，选择时终端会输出点的坐标
注意：至少需要3个顶点，且框选区域需要是凸多边形
再按下s，剔除框选区域的点并保存"output_pcd_file".pcd文件在build目录下
可反复框选区域删除
按q退出程序