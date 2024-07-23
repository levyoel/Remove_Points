#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// 读取配置文件
json readConfig(const std::string& config_file) {
    std::ifstream file(config_file); // 打开配置文件
    if (!file.is_open()) {
        std::cerr << "Could not open config file: " << config_file << std::endl;
        exit(EXIT_FAILURE);
    }

    json config;
    file >> config; // 读取文件内容到JSON对象
    return config;
}

std::vector<pcl::PointIndices::Ptr> selected_polygons; // 存储选择的多边形区域的点索引
pcl::PointIndices::Ptr current_polygon(new pcl::PointIndices); // 当前多边形区域的点索引
bool is_selecting = false; // 是否正在选择标志
std::vector<PointT> selected_points; // 存储选择的点
float z_height = 0.0f; // 多边形选择区域的高度

// 判断点是否在多边形内的函数
bool isPointInPolygon(const PointT& point, const std::vector<PointT>& polygon) {
    int i, j, nvert = polygon.size();
    bool c = false;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((polygon[i].y >= point.y) != (polygon[j].y >= point.y)) &&
            (point.x <= (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)) {
            c = !c;
        }
    }
    // 检查点是否在 Z 轴高度范围内
    return c && (point.z >= -z_height / 2 && point.z <= z_height / 2);
}

// 剔除选择区域内的点并保存
void removeSelectedPoints(PointCloudT::Ptr cloud, const std::vector<pcl::PointIndices::Ptr>& selected_polygons, const std::string& output_file) {
    if (!cloud || cloud->points.empty()) {
        std::cerr << "Error: cloud is null or empty." << std::endl;
        return;
    }

    pcl::PointIndices::Ptr all_indices(new pcl::PointIndices); // 所有要删除的点的索引
    for (const auto& indices : selected_polygons) {
        all_indices->indices.insert(all_indices->indices.end(), indices->indices.begin(), indices->indices.end());
    }

    if (all_indices->indices.empty()) {
        std::cerr << "Error: No indices to remove." << std::endl;
        return;
    }

    pcl::ExtractIndices<PointT> extract; // 创建索引提取器
    extract.setInputCloud(cloud); // 设置输入点云
    extract.setIndices(all_indices); // 设置要删除的点索引
    extract.setNegative(true); // 删除选中的点
    PointCloudT::Ptr filtered_cloud(new PointCloudT); // 创建过滤后的点云
    extract.filter(*filtered_cloud); // 执行过滤操作
    *cloud = *filtered_cloud; // 更新原始点云

    // 保存过滤后的点云到文件
    if (pcl::io::savePCDFileASCII(output_file, *cloud) == -1) {
        PCL_ERROR("Couldn't write file %s\n", output_file.c_str());
    } else {
        std::cout << "Saved filtered point cloud to " << output_file << std::endl;
    }
}

// 鼠标选点回调函数
void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
    if (is_selecting) {
        float x, y, z;
        event.getPoint(x, y, z); // 获取选择点的坐标
        selected_points.push_back(PointT(x, y, z)); // 将点添加到选中点列表中
        std::cout << "Point selected: (" << x << ", " << y << ", " << z << ")" << std::endl;
    }
}

// 键盘回调函数
void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
    PointCloudT::Ptr cloud = *static_cast<PointCloudT::Ptr*>(viewer_void);
    if (!cloud) {
        std::cerr << "Error: cloud is null." << std::endl;
        return;
    }

    if (event.getKeySym() == "s" && event.keyDown()) {
        if (!is_selecting) {
            is_selecting = true; // 开始选择
            selected_points.clear(); // 清除之前选择的点
            current_polygon.reset(new pcl::PointIndices); // 重置当前多边形
            std::cout << "Selection started." << std::endl;
        } else {
            is_selecting = false; // 结束选择
            if (selected_points.size() > 2) {
                std::cout << "Selection finished. Polygon with " << selected_points.size() << " vertices." << std::endl;

                for (size_t i = 0; i < cloud->points.size(); ++i) {
                    if (isPointInPolygon(cloud->points[i], selected_points)) {
                        current_polygon->indices.push_back(i); // 添加点索引到当前多边形
                    }
                }

                if (current_polygon->indices.size() > 0) {
                    selected_polygons.push_back(current_polygon); // 保存当前多边形
                }
            } else {
                std::cerr << "Error: At least three points are required to define a selection polygon." << std::endl;
            }
        }
    }
    if (event.getKeySym() == "d" && event.keyDown()) {
        selected_polygons.clear(); // 清除所有选中的多边形
        std::cout << "All selections cleared." << std::endl;
    }

    // 保存剔除后的点云到文件并退出
    if (event.getKeySym() == "q" && event.keyDown()) {
        json config = readConfig("config.json"); // 读取配置文件
        std::string output_file = config["output_pcd_file"];
        removeSelectedPoints(cloud, selected_polygons, output_file); // 删除选中的点并保存文件
        std::cout << "saving succeed." << std::endl;
    }
}

int main(int argc, char** argv) {
    json config = readConfig("config.json");
    std::string input_file = config["input_pcd_file"];
    std::vector<int> background_color = config["background_color"];
    int point_size = config["point_size"];
    z_height = config["z_height"];

    PointCloudT::Ptr cloud(new PointCloudT);

    if (pcl::io::loadPCDFile<PointT>(input_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", input_file.c_str()); // 读取点云文件
        return (-1);
    }

    std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(background_color[0], background_color[1], background_color[2]); // 设置背景颜色
    viewer->addPointCloud<PointT>(cloud, "sample cloud"); // 添加点云
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "sample cloud"); // 设置点大小

    viewer->registerPointPickingCallback(pointPickingCallback, (void*)cloud.get()); // 注册点选回调函数
    viewer->registerKeyboardCallback(keyboardCallback, (void*)&cloud); // 注册键盘回调函数

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100); // 更新视图

        if (!is_selecting && !selected_polygons.empty()) {
            removeSelectedPoints(cloud, selected_polygons,config["output_pcd_file"]);
            viewer->removePointCloud("sample cloud");
            viewer->addPointCloud<PointT>(cloud, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "sample cloud");
            selected_polygons.clear(); // 本次操作过后，清空选择的多边形顶点信息
        }
    }

    return 0;
}
