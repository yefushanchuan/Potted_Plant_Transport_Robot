// kf_extract.cpp
// 关键帧提取工具：从建图保存的 patches + poses.txt 提取关键帧数据库
//
// 用法:
//   kf_extract <pcd_dir> [options]
//
//   <pcd_dir> 含 patches/ + poses.txt 的目录，或其父目录（自动找最新时间戳子目录）
//   输出到 <pcd_dir> 下：keyframes.bin, keyframe_poses.txt, clouds/
//
// 选项:
//   --origin x y z        重定义原点（位姿同步平移）
//   --distance 3.0        关键帧间距阈值(米)，默认3
//   --radius 25.0         SC 矩阵最大半径(米)，默认25
//   --bins 360            极坐标环 bin 数，默认360
//   --z_min 0.2           BEV z 下界，默认0.2
//   --z_max 2.0           BEV z 上界，默认2.0
//   --leaf 0.1            体素滤波叶子大小(米)，默认0.1

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

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

struct Keyframe {
    int id;
    double x, y, z;
    double qw, qx, qy, qz;
    std::vector<float> polar_ring;     // 极坐标环特征 (N 维)
    std::vector<float> polar_fft_mag;  // PolarRing FFT 幅度 (N/2 维, 旋转不变)
    std::vector<float> sc_matrix;      // 完整 SC 矩阵 (20×120=2400, row-major)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

struct PoseEntry {
    std::string filename;
    double x, y, z;
    double qw, qx, qy, qz;
};

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

bool isFarEnough(const PoseEntry &candidate,
                 const std::vector<Keyframe> &selected,
                 double min_dist) {
    for (const auto &kf : selected) {
        double dx = candidate.x - kf.x;
        double dy = candidate.y - kf.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < min_dist) return false;
    }
    return true;
}

std::string resolvePcdDir(const std::string &input) {
    fs::path p(input);
    if (!fs::exists(p) || !fs::is_directory(p)) {
        std::cerr << "路径不存在或不是目录: " << input << "\n";
        return {};
    }
    // 直接含 patches/ + poses.txt
    if (fs::exists(p / "patches") && fs::exists(p / "poses.txt")) {
        return p.string();
    }
    // 扫描子目录，找最新的时间戳文件夹
    std::vector<fs::path> subdirs;
    for (auto &entry : fs::directory_iterator(p)) {
        if (entry.is_directory() &&
            fs::exists(entry.path() / "patches") &&
            fs::exists(entry.path() / "poses.txt")) {
            subdirs.push_back(entry.path());
        }
    }
    if (subdirs.empty()) {
        std::cerr << "未找到含 patches/ + poses.txt 的子目录: " << input << "\n";
        return {};
    }
    std::sort(subdirs.begin(), subdirs.end());
    return subdirs.back().string();
}

void printUsage() {
    std::cout << "用法: kf_extract <pcd_dir> [选项]\n"
              << "  <pcd_dir>  含 patches/ + poses.txt 的目录，或其父目录（自动找最新）\n"
              << "  输出到 <pcd_dir> 下\n\n"
              << "选项:\n"
              << "  --origin x y z      重定义原点（位姿同步平移）\n"
              << "  --distance 3.0      关键帧间距阈值(米)\n"
              << "  --radius 25.0       SC 矩阵最大半径(米)\n"
              << "  --bins 360          极坐标环 bin 数\n"
              << "  --z_min 0.2         BEV z 下界\n"
              << "  --z_max 2.0         BEV z 上界\n"
              << "  --leaf 0.1          体素滤波叶子大小(米)\n";
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }

    std::string input_path = argv[1];

    // 默认参数
    double origin_x = 0, origin_y = 0, origin_z = 0;
    bool use_origin = false;
    double min_distance = 3.0;
    double sc_radius = 25.0;
    int ring_bins = 360;
    float z_min = 0.2f, z_max = 2.0f;
    float leaf_size = 0.1f;

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--origin" && i + 3 < argc) {
            origin_x = std::stod(argv[++i]);
            origin_y = std::stod(argv[++i]);
            origin_z = std::stod(argv[++i]);
            use_origin = true;
        } else if (arg == "--distance" && i + 1 < argc) {
            min_distance = std::stod(argv[++i]);
        } else if (arg == "--radius" && i + 1 < argc) {
            sc_radius = std::stod(argv[++i]);
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

    // 解析 pcd 目录
    std::string pcd_dir = resolvePcdDir(input_path);
    if (pcd_dir.empty()) return 1;
    std::string patches_dir = pcd_dir + "/patches";
    std::string poses_path = pcd_dir + "/poses.txt";
    std::string clouds_dir = pcd_dir + "/clouds";
    std::string bin_path = pcd_dir + "/keyframes.bin";
    std::string poses_out_path = pcd_dir + "/keyframe_poses.txt";

    std::cout << "pcd 目录: " << pcd_dir << "\n";

    // 加载位姿
    std::cout << "加载位姿: " << poses_path << " ... ";
    std::cout.flush();
    auto poses = loadPoses(poses_path);
    std::cout << poses.size() << " 帧\n";

    // 筛选关键帧
    std::cout << "筛选关键帧 (间距>=" << min_distance << "m) ...\n";
    std::vector<Keyframe> keyframes;

    for (size_t pose_idx = 0; pose_idx < poses.size(); pose_idx++) {
        const auto &pose = poses[pose_idx];

        double px = pose.x, py = pose.y, pz = pose.z;
        Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
        if (use_origin) {
            px -= origin_x;
            py -= origin_y;
            pz -= origin_z;
        }

        // 距离筛选
        PoseEntry adjusted = pose;
        adjusted.x = px;
        adjusted.y = py;
        adjusted.z = pz;
        if (!isFarEnough(adjusted, keyframes, min_distance)) continue;

        // 加载 patch (body/IMU 帧)
        std::string patch_path = patches_dir + "/" + pose.filename;
        if (!fs::exists(patch_path)) {
            std::cout << "  跳过 " << pose.filename << " (文件不存在)\n";
            continue;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr patch(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile(patch_path, *patch) < 0) {
            std::cout << "  跳过 " << pose.filename << " (加载失败)\n";
            continue;
        }

        if (patch->size() < 50) {
            std::cout << "  跳过 " << pose.filename << " (点云太少: " << patch->size() << ")\n";
            continue;
        }

        // 变换到 world 帧
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
        T(0, 3) = static_cast<float>(px);
        T(1, 3) = static_cast<float>(py);
        T(2, 3) = static_cast<float>(pz);

        pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*patch, *world_cloud, T);

        // 体素滤波
        if (leaf_size > 0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setLeafSize(leaf_size, leaf_size, leaf_size);
            vg.setInputCloud(world_cloud);
            vg.filter(*filtered);
            world_cloud = filtered;
        }

        if (world_cloud->size() < 50) {
            std::cout << "  跳过 " << pose.filename << " (滤波后点云太少: " << world_cloud->size() << ")\n";
            continue;
        }

        // 特征提取：平移到关键帧相对坐标
        pcl::PointCloud<pcl::PointXYZI>::Ptr local(new pcl::PointCloud<pcl::PointXYZI>);
        local->resize(world_cloud->size());
        for (size_t j = 0; j < world_cloud->size(); j++) {
            (*local)[j] = (*world_cloud)[j];
            (*local)[j].x -= static_cast<float>(px);
            (*local)[j].y -= static_cast<float>(py);
            (*local)[j].z -= static_cast<float>(pz);
        }

        auto ring = extractPolarRing(local, ring_bins, z_min, z_max);

        // PolarRing FFT 幅度
        int fft_mag_size = ring_bins / 2;
        std::vector<float> polar_fft_mag(fft_mag_size);
        for (int k = 0; k < fft_mag_size; k++) {
            float real = 0.0f, imag = 0.0f;
            for (int n = 0; n < ring_bins; n++) {
                float angle = -2.0f * M_PI * k * n / ring_bins;
                real += ring[n] * std::cos(angle);
                imag += ring[n] * std::sin(angle);
            }
            polar_fft_mag[k] = std::sqrt(real * real + imag * imag) / ring_bins;
        }

        // SC 矩阵 (20×120, row-major)
        int sc_num_ring = 20, sc_num_sector = 120;
        std::vector<float> sc_matrix(sc_num_ring * sc_num_sector, -1000.0f);
        {
            float sc_max_radius = static_cast<float>(sc_radius);
            float sc_z_min = 0.1f;
            for (const auto &pt : local->points) {
                float rho = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                if (rho > sc_max_radius || pt.z < sc_z_min) continue;
                float theta = std::atan2(pt.y, pt.x);
                if (theta < 0) theta += 2.0f * M_PI;
                int ring_idx = std::min(sc_num_ring - 1,
                    static_cast<int>(rho / sc_max_radius * sc_num_ring));
                int sector_idx = std::min(sc_num_sector - 1,
                    static_cast<int>(theta / (2.0f * M_PI) * sc_num_sector));
                int idx = ring_idx * sc_num_sector + sector_idx;
                if (pt.z > sc_matrix[idx]) sc_matrix[idx] = pt.z;
            }
            for (float &v : sc_matrix) { if (v < -999.0f) v = 0.0f; }
        }

        Keyframe kf;
        kf.id = static_cast<int>(keyframes.size());
        kf.x = px;
        kf.y = py;
        kf.z = pz;
        kf.qw = pose.qw;
        kf.qx = pose.qx;
        kf.qy = pose.qy;
        kf.qz = pose.qz;
        kf.polar_ring = ring;
        kf.polar_fft_mag = polar_fft_mag;
        kf.sc_matrix = sc_matrix;
        kf.cloud = world_cloud;
        keyframes.push_back(kf);

        std::cout << "  [" << keyframes.size() - 1 << "] " << pose.filename
                  << " pos=(" << px << ", " << py << ", " << pz << ")"
                  << " pts=" << world_cloud->size() << "\n";
    }

    std::cout << "选中 " << keyframes.size() << " 个关键帧\n";

    // 创建输出目录
    fs::create_directories(clouds_dir);

    // 保存元数据（二进制）
    std::cout << "保存 keyframes.bin ... ";
    std::cout.flush();
    {
        std::ofstream meta_f(bin_path, std::ios::binary);
        uint32_t count = keyframes.size();
        int32_t sc_ring_out = 20, sc_sector_out = 120;
        float sc_max_radius_out = static_cast<float>(sc_radius);
        uint32_t magic = 0x44445343;

        meta_f.write(reinterpret_cast<const char *>(&magic), sizeof(magic));
        meta_f.write(reinterpret_cast<const char *>(&count), sizeof(count));
        meta_f.write(reinterpret_cast<const char *>(&ring_bins), sizeof(ring_bins));
        meta_f.write(reinterpret_cast<const char *>(&sc_ring_out), sizeof(sc_ring_out));
        meta_f.write(reinterpret_cast<const char *>(&sc_sector_out), sizeof(sc_sector_out));
        meta_f.write(reinterpret_cast<const char *>(&sc_max_radius_out), sizeof(sc_max_radius_out));

        for (size_t i = 0; i < keyframes.size(); i++) {
            const auto &kf = keyframes[i];
            meta_f.write(reinterpret_cast<const char *>(&kf.x), sizeof(double));
            meta_f.write(reinterpret_cast<const char *>(&kf.y), sizeof(double));
            meta_f.write(reinterpret_cast<const char *>(&kf.z), sizeof(double));
            meta_f.write(reinterpret_cast<const char *>(&kf.qw), sizeof(double));
            meta_f.write(reinterpret_cast<const char *>(&kf.qx), sizeof(double));
            meta_f.write(reinterpret_cast<const char *>(&kf.qy), sizeof(double));
            meta_f.write(reinterpret_cast<const char *>(&kf.qz), sizeof(double));
            meta_f.write(reinterpret_cast<const char *>(kf.polar_fft_mag.data()),
                       (ring_bins / 2) * sizeof(float));
            meta_f.write(reinterpret_cast<const char *>(kf.sc_matrix.data()),
                       sc_ring_out * sc_sector_out * sizeof(float));
        }
    }
    std::cout << "done\n";

    // 保存点云
    std::cout << "保存 clouds/*.pcd ... ";
    std::cout.flush();
    for (size_t i = 0; i < keyframes.size(); i++) {
        std::string pcd_path = clouds_dir + "/" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_path, *keyframes[i].cloud);
    }
    std::cout << "done\n";

    // 保存文本位姿
    std::cout << "保存 keyframe_poses.txt ... ";
    std::cout.flush();
    {
        std::ofstream poses_f(poses_out_path);
        poses_f << "# index filename x y z qw qx qy qz\n";
        for (size_t i = 0; i < keyframes.size(); i++) {
            const auto &kf = keyframes[i];
            poses_f << i << " " << i << ".pcd "
                    << kf.x << " " << kf.y << " " << kf.z << " "
                    << kf.qw << " " << kf.qx << " " << kf.qy << " " << kf.qz << "\n";
        }
    }
    std::cout << "done\n";

    std::cout << "\n关键帧数据库已保存到: " << pcd_dir << "\n"
              << "  keyframes.bin       元数据（位姿+SC+FFT）\n"
              << "  keyframe_poses.txt  位姿文本\n"
              << "  clouds/*.pcd        各关键帧点云 (world 帧)\n";

    return 0;
}
