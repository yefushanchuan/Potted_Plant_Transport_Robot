// kf_extract.cpp
// 关键帧提取工具：从处理后的地图 + 建图位姿 提取关键帧数据库
//
// 用法:
//   kf_extract <map.pcd> <poses.txt> <output_dir> [options]
//
// 选项:
//   --origin x y z        重定义原点（地图和位姿同步平移）
//   --distance 10.0       关键帧间距阈值(米)，默认10
//   --radius 20.0         每个关键帧裁剪半径(米)，默认20
//   --bins 360            极坐标环 bin 数，默认360
//   --z_min -0.5          BEV z 下界
//   --z_max 2.0           BEV z 上界
//   --leaf 0.1            体素滤波叶子大小(米)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

struct Keyframe {
    double x, y, z;
    double qw, qx, qy, qz;
    std::vector<float> ring;  // 极坐标环特征
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

struct PoseEntry {
    std::string filename;
    double x, y, z;
    double qw, qx, qy, qz;
};

// 解析 poses.txt
std::vector<PoseEntry> loadPoses(const std::string &path) {
    std::vector<PoseEntry> poses;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        PoseEntry p;
        ss >> p.filename >> p.x >> p.y >> p.z >> p.qw >> p.qx >> p.qy >> p.qz;
        poses.push_back(p);
    }
    return poses;
}

// 极坐标环特征提取
std::vector<float> extractPolarRing(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                     int num_bins, float z_min, float z_max) {
    std::vector<float> ring(num_bins, 0.0f);
    float bin_angle = 2.0f * M_PI / num_bins;

    for (const auto &pt : cloud->points) {
        if (pt.z < z_min || pt.z > z_max) continue;
        float rho = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        float theta = std::atan2(pt.y, pt.x);
        if (theta < 0) theta += 2.0f * M_PI;
        int bin = static_cast<int>(theta / bin_angle) % num_bins;
        if (rho > ring[bin]) ring[bin] = rho;
    }
    return ring;
}

// 距离筛选：与已选关键帧保持最小距离
bool isFarEnough(const PoseEntry &candidate,
                 const std::vector<Keyframe> &selected,
                 double min_dist) {
    double min_d = min_dist;
    for (const auto &kf : selected) {
        double dx = candidate.x - kf.x;
        double dy = candidate.y - kf.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < min_d) return false;
    }
    return true;
}

void printUsage() {
    std::cout << "用法: kf_extract <map.pcd> <poses.txt> <output_dir> [选项]\n"
              << "选项:\n"
              << "  --origin x y z      重定义原点（地图和位姿同步平移）\n"
              << "  --distance 3.0      关键帧间距阈值(米)\n"
              << "  --radius 25.0       每个关键帧裁剪半径(米)\n"
              << "  --bins 360          极坐标环 bin 数\n"
              << "  --z_min -0.5        BEV z 下界\n"
              << "  --z_max 2.0         BEV z 上界\n"
              << "  --leaf 0.1          体素滤波叶子大小(米)\n";
}

int main(int argc, char **argv) {
    if (argc < 4) {
        printUsage();
        return 1;
    }

    std::string map_path = argv[1];
    std::string poses_path = argv[2];
    std::string output_dir = argv[3];

    // 默认参数
    double origin_x = 0, origin_y = 0, origin_z = 0;
    bool use_origin = false;
    double min_distance = 3.0;
    double crop_radius = 25.0;
    int ring_bins = 360;
    float z_min = 0.2f, z_max = 2.0f;
    float leaf_size = 0.1f;

    // 解析选项
    for (int i = 4; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--origin" && i + 3 < argc) {
            origin_x = std::stod(argv[++i]);
            origin_y = std::stod(argv[++i]);
            origin_z = std::stod(argv[++i]);
            use_origin = true;
        } else if (arg == "--distance" && i + 1 < argc) {
            min_distance = std::stod(argv[++i]);
        } else if (arg == "--radius" && i + 1 < argc) {
            crop_radius = std::stod(argv[++i]);
        } else if (arg == "--bins" && i + 1 < argc) {
            ring_bins = std::stoi(argv[++i]);
        } else if (arg == "--z_min" && i + 1 < argc) {
            z_min = std::stof(argv[++i]);
        } else if (arg == "--z_max" && i + 1 < argc) {
            z_max = std::stof(argv[++i]);
        } else if (arg == "--leaf" && i + 1 < argc) {
            leaf_size = std::stof(argv[++i]);
        } else {
            std::cerr << "未知参数: " << arg << "\n";
            printUsage();
            return 1;
        }
    }

    // 加载地图
    std::cout << "加载地图: " << map_path << " ... ";
    std::cout.flush();
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile(map_path, *map_cloud) < 0) {
        std::cerr << "无法加载地图: " << map_path << "\n";
        return 1;
    }
    std::cout << map_cloud->size() << " 点\n";

    // 平移地图（重定义原点）
    if (use_origin) {
        std::cout << "重定义原点: (" << origin_x << ", " << origin_y << ", " << origin_z << ")\n";
        for (auto &pt : map_cloud->points) {
            pt.x -= origin_x;
            pt.y -= origin_y;
            pt.z -= origin_z;
        }
    }

    // 加载位姿
    std::cout << "加载位姿: " << poses_path << " ... ";
    std::cout.flush();
    auto poses = loadPoses(poses_path);
    std::cout << poses.size() << " 帧\n";

    // 体素滤波地图
    std::cout << "体素滤波 (leaf=" << leaf_size << "m) ... ";
    std::cout.flush();
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.setInputCloud(map_cloud);
    vg.filter(*map_filtered);
    std::cout << map_filtered->size() << " 点\n";

    // 筛选关键帧
    std::cout << "筛选关键帧 (间距>=" << min_distance << "m) ...\n";
    std::vector<Keyframe> keyframes;

    for (const auto &pose : poses) {
        // 平移位姿
        double px = pose.x, py = pose.y, pz = pose.z;
        if (use_origin) {
            px -= origin_x;
            py -= origin_y;
            pz -= origin_z;
        }

        // 检查是否与已选关键帧距离足够
        PoseEntry adjusted = pose;
        adjusted.x = px;
        adjusted.y = py;
        adjusted.z = pz;
        if (!isFarEnough(adjusted, keyframes, min_distance)) continue;

        // 裁剪局部区域
        float r2 = crop_radius * crop_radius;
        pcl::PointCloud<pcl::PointXYZI>::Ptr local(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto &pt : map_filtered->points) {
            float dx = pt.x - px;
            float dy = pt.y - py;
            if (dx * dx + dy * dy < r2) {
                local->push_back(pt);
            }
        }

        if (local->size() < 50) {
            std::cout << "  跳过 " << pose.filename << " (局部点云太少: " << local->size() << ")\n";
            continue;
        }

        // 计算环特征
        auto ring = extractPolarRing(local, ring_bins, z_min, z_max);

        Keyframe kf;
        kf.x = px;
        kf.y = py;
        kf.z = pz;
        kf.qw = pose.qw;
        kf.qx = pose.qx;
        kf.qy = pose.qy;
        kf.qz = pose.qz;
        kf.ring = ring;
        kf.cloud = local;
        keyframes.push_back(kf);
    }

    std::cout << "选中 " << keyframes.size() << " 个关键帧\n";

    // 保存
    fs::create_directories(output_dir);
    fs::create_directories(output_dir + "/clouds");

    // 保存元数据（二进制）
    std::string meta_path = output_dir + "/keyframes.bin";
    std::ofstream meta_f(meta_path, std::ios::binary);

    uint32_t count = keyframes.size();
    meta_f.write(reinterpret_cast<const char *>(&count), sizeof(count));
    meta_f.write(reinterpret_cast<const char *>(&ring_bins), sizeof(ring_bins));

    for (size_t i = 0; i < keyframes.size(); i++) {
        const auto &kf = keyframes[i];
        // 位姿
        meta_f.write(reinterpret_cast<const char *>(&kf.x), sizeof(double));
        meta_f.write(reinterpret_cast<const char *>(&kf.y), sizeof(double));
        meta_f.write(reinterpret_cast<const char *>(&kf.z), sizeof(double));
        meta_f.write(reinterpret_cast<const char *>(&kf.qw), sizeof(double));
        meta_f.write(reinterpret_cast<const char *>(&kf.qx), sizeof(double));
        meta_f.write(reinterpret_cast<const char *>(&kf.qy), sizeof(double));
        meta_f.write(reinterpret_cast<const char *>(&kf.qz), sizeof(double));
        // 环特征
        meta_f.write(reinterpret_cast<const char *>(kf.ring.data()),
                     ring_bins * sizeof(float));
    }
    meta_f.close();

    // 保存点云
    for (size_t i = 0; i < keyframes.size(); i++) {
        std::string pcd_path = output_dir + "/clouds/" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_path, *keyframes[i].cloud);
    }

    // 保存文本格式的位姿（方便查看）
    std::string poses_out = output_dir + "/keyframe_poses.txt";
    std::ofstream poses_f(poses_out);
    poses_f << "# index filename x y z qw qx qy qz\n";
    for (size_t i = 0; i < keyframes.size(); i++) {
        const auto &kf = keyframes[i];
        poses_f << i << " " << i << ".pcd "
                << kf.x << " " << kf.y << " " << kf.z << " "
                << kf.qw << " " << kf.qx << " " << kf.qy << " " << kf.qz << "\n";
    }
    poses_f.close();

    std::cout << "\n关键帧数据库已保存到: " << output_dir << "\n"
              << "  keyframes.bin       元数据（位姿+环特征）\n"
              << "  keyframe_poses.txt  位姿文本\n"
              << "  clouds/*.pcd        各关键帧点云\n";

    return 0;
}
