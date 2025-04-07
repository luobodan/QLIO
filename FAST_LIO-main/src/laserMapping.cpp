// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <filesystem> // C++17 引入的文件系统库
#include <sys/types.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <unordered_map>
#include <random>
#include <mutex>
#include <atomic>
#include <sstream>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
string save_dir;
// 定义RGB点云类型
using RGBPointType = pcl::PointXYZRGB;
/*** Time Log Variables ***/
void createDirectoryIfNotExists(const std::string &dir);
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
double dis_scale = 0.05;
int bit_disr=2;
bool save_pcd = true;
/**************************/
ros::Publisher pubLaserCloudEffect;
//qbggin
int max_points_all = 500;
int down_sample_times = 5;
bool use_resample;
bool float_or_fixed = true;
struct Point {
    double x, y, z;
};
class PointCloudCompressor {
private:
    int bits;
    double min_val, max_val;

    std::string floatToBinary(float num) const {
        uint32_t binary;
        std::memcpy(&binary, &num, sizeof(float));
        return std::bitset<32>(binary).to_string();
    }

    float binaryToFloat(const std::string& binary) const {
        uint32_t intValue = std::bitset<32>(binary).to_ulong();
        float result;
        std::memcpy(&result, &intValue, sizeof(float));
        return result;
    }

    // 根据总位数计算指数位数
    int calculateExponentBits(int totalBits) const {
        if (totalBits <= 8) return 3;
        if (totalBits <= 10) return 4;
        if (totalBits <= 12) return 5;
        if (totalBits <= 14) return 6;
        return 8; // 标准单精度浮点数的指数位数
    }

public:
    PointCloudCompressor(int bits = 12, double min_val = -200.0, double max_val = 200.0)
        : bits(bits), min_val(min_val), max_val(max_val) {
        if (bits < 6 || bits > 16) {
            throw std::invalid_argument("Bits must be between 6 and 16");
        }
    }
    void set_bits(int bits_ = 12)
    {
        if (bits < 2 || bits > 16) {
            throw std::invalid_argument("Bits must be between 6 and 16");
        }
        std::cout<<"set bits as "<< bits_ <<" bits is "<< bits << std::endl;
        bits = bits_;
        std::cout<<"set bits as "<< bits_ <<" bits is "<< bits << std::endl;
    }
    float quantizeFloat(float value) const {
        std::string binary = floatToBinary(value);
        int exponentBits = calculateExponentBits(bits);
        int mantissaBits = bits - exponentBits - 1; // 1 bit for sign

        std::string sign = binary.substr(0, 1);
        std::string exponent = binary.substr(1, exponentBits);
        std::string mantissa = binary.substr(9, mantissaBits);

        // 填充剩余的位
        mantissa += std::string(23 - mantissaBits, '0');
        exponent = std::string(8 - exponentBits, '0') + exponent;

        return binaryToFloat(sign + exponent + mantissa);
    }

    float quantizeFixedRange(float value) const {
        double range_size = max_val - min_val;
        double step_size = range_size / ((1 << bits) - 1);
        int quantized = std::round((value - min_val) / step_size);
        quantized = std::max(0, std::min(quantized, (1 << bits) - 1));
        return quantized * step_size + min_val;
    }

    std::vector<Point> processPointCloudFloat(const std::vector<Point>& points) const {
        std::vector<Point> result;
        result.reserve(points.size());
        for (const auto& point : points) {
            result.push_back({
                quantizeFloat(static_cast<float>(point.x)),
                quantizeFloat(static_cast<float>(point.y)),
                quantizeFloat(static_cast<float>(point.z))
            });
        }
        return result;
    }

    std::vector<Point> processPointCloudFixedRange(const std::vector<Point>& points) const {
        std::vector<Point> result;
        result.reserve(points.size());
        for (const auto& point : points) {
            result.push_back({
                quantizeFixedRange(static_cast<float>(point.x)),
                quantizeFixedRange(static_cast<float>(point.y)),
                quantizeFixedRange(static_cast<float>(point.z))
            });
        }
        return result;
    }
};

void printPoint(const Point& p) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")";
}
PointCloudCompressor pointCompressor(8);
// int main() {
//     std::vector<Point> original_points = {
//         {100.5, -150.3, 75.2},
//         {-180.1, 20.7, -90.6},
//         {0.0, 199.9, -199.9},
//         {30.0, 49.9, -129.9},
//         {40.5, 59.9, -39.9},
//         {70.5, 2.9, -5.9}
//     };

//     std::cout << "Original points:" << std::endl;
//     for (const auto& p : original_points) {
//         printPoint(p);
//         std::cout << std::endl;
//     }

//     for (int bits = 6; bits <= 16; ++bits) {
//         PointCloudCompressor compressor(bits);

//         std::cout << "\n=== Using " << bits << " bits ===" << std::endl;

//         // 定点量化
//         std::cout << "\nFixed-point quantization:" << std::endl;
//         auto quantized_fixed = compressor.processPointCloudFixedRange(original_points);
            // float_or_fixed
//         // 浮点量化
//         std::cout << "\nFloating-point quantization:" << std::endl;
//         auto quantized_float = compressor.processPointCloudFloat(original_points);

//         for (size_t i = 0; i < original_points.size(); ++i) {
//             std::cout << "Original: ";
//             printPoint(original_points[i]);
//             std::cout << std::endl;
            
//             std::cout << "Fixed: ";
//             printPoint(quantized_fixed[i]);
//             std::cout << std::endl;
            
//             std::cout << "Float: ";
//             printPoint(quantized_float[i]);
//             std::cout << std::endl << std::endl;
//         }
//     }

//     return 0;
// }
//qover
void createDirectoryIfNotExists(const std::string &dir) {
    boost::filesystem::path path(dir);

    // 检查目录是否存在
    if (!boost::filesystem::exists(path)) {
        std::cout << "Directory does not exist, creating it: " << dir << std::endl;
        // 创建目录及其父目录（如果需要）
        if (!boost::filesystem::create_directories(path)) {
            std::cerr << "Failed to create directory: " << dir << std::endl;
        }
    } else {
        std::cout << "Directory already exists: " << dir << std::endl;
    }
}

void appendPoseToTUM(const std::string& filename, double timestamp, const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
    std::ofstream file(filename, std::ios::app); // 使用追加模式打开文件
    if (!file.is_open()) {
        std::cerr << "Unable to open or create file: " << filename << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6)
         << timestamp << " "
         << t.x() << " " << t.y() << " " << t.z() << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    
    file.close();
}
float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);


bool use_r = false;
bool use_zr = false;
int  bits_r = 1;
int  bits_p = 6;
float thr_ = 0.4;
float thr_2 = 0.6;
float thr_3 = 0.9;
float thr_4 = 1;
float quantizationLevel = 0.01;
/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void rRGBpointBodyToWorld(RGBPointType const * const pi, RGBPointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->rgba = pi->rgba;
    // po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();

    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer11");
        lidar_buffer.clear();
    }
        // ROS_ERROR("lidar loop back, clear buffer11");
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

        // ROS_ERROR("lidar loop back, clear buffer22");

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{

    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    // 保存位姿到指定的文件
    std::string save_txt = save_dir + "/fastlio_tum.txt";
    std::ofstream save_file;
    save_file.open(save_txt, std::ios::app | std::ios::out); // 以追加模式打开文件

    if (!save_file.is_open()) {
        ROS_ERROR("无法打开文件: %s", save_txt.c_str());
        return;
    }

    // 设置输出精度
    save_file << std::fixed << std::setprecision(9);

    // 获取四元数
    tf::Quaternion q_save;
    q_save.setW(odomAftMapped.pose.pose.orientation.w);
    q_save.setX(odomAftMapped.pose.pose.orientation.x);
    q_save.setY(odomAftMapped.pose.pose.orientation.y);
    q_save.setZ(odomAftMapped.pose.pose.orientation.z);

    // 保存 TUM 格式的位姿
    save_file << odomAftMapped.header.stamp.toSec() << " "
              << odomAftMapped.pose.pose.position.x << " "
              << odomAftMapped.pose.pose.position.y << " "
              << odomAftMapped.pose.pose.position.z << " "
              << q_save.x() << " "
              << q_save.y() << " "
              << q_save.z() << " "
              << q_save.w() << "\n";

    save_file.close();
    //save_dir
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}
//
// void normalizeAndQuantize(NormVec* normvec, int i, double quantizationLevel = 0.01) {
//     // 量化 x 分量
//     normvec->points[i].x = std::round(normvec->points[i].x / quantizationLevel) * quantizationLevel;
    
//     // 量化 y 分量
//     normvec->points[i].y = std::round(normvec->points[i].y / quantizationLevel) * quantizationLevel;
    
//     // 量化 z 分量
//     normvec->points[i].z = std::round(normvec->points[i].z / quantizationLevel) * quantizationLevel;
    
//     // 重新归一化向量
//     double length = std::sqrt(normvec->points[i].x * normvec->points[i].x + 
//                               normvec->points[i].y * normvec->points[i].y + 
//                               normvec->points[i].z * normvec->points[i].z);
    
//     normvec->points[i].x /= length;
//     normvec->points[i].y /= length;
//     normvec->points[i].z /= length;
// }
int len_d_;// = static_cast<int>(1 / quantizationLevel); // 3 * len_dir_
int every_vect_max;

struct Point3D {
    double x, y, z;
    int original_index;
};

struct IntervalInfo {
    std::vector<Point3D> points;
    V3D normal;
    int r;
    int g;
    int b;
};
struct V3I {
    int x, y, z;
    V3I(int x, int y, int z) : x(x), y(y), z(z) {}
    bool operator==(const V3I& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};
struct V3IHash {
    std::size_t operator()(const V3I& k) const {
        return std::hash<int>()(k.x) ^ std::hash<int>()(k.y) ^ std::hash<int>()(k.z);
    }
};
// vector<int> count_len;

void hsvToRgb(float h, float s, float v, int& r, int& g, int& b) {
    h = std::fmod(h, 1.0f);
    if (h < 0) h += 1.0f;
    
    const int i = static_cast<int>(h * 6);
    const float f = h * 6 - i;
    const float p = v * (1 - s);
    const float q = v * (1 - f * s);
    const float t = v * (1 - (1 - f) * s);

    float red, green, blue;
    switch (i % 6) {
        case 0: red = v; green = t; blue = p; break;
        case 1: red = q; green = v; blue = p; break;
        case 2: red = p; green = v; blue = t; break;
        case 3: red = p; green = q; blue = v; break;
        case 4: red = t; green = p; blue = v; break;
        case 5: red = v; green = p; blue = q; break;
    }

    r = static_cast<int>(red * 255);
    g = static_cast<int>(green * 255);
    b = static_cast<int>(blue * 255);
}

ros::Publisher rgb_cloud_pub;
ros::Publisher rgb_chosen_cloud_pub, rgb_chosen_compressed_cloud_pub;
PointCloudXYZI::Ptr laserCloudWorld_1(new PointCloudXYZI());
// RViz专用颜色生成（优化gamma值）
void rvizHSVtoRGB(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
    h = fmod(h, 1.0f);
    const float gamma = 2.2f; // 针对RViz显示优化
    float c = v * s;
    float x = c * (1 - fabs(fmod(6 * h, 2) - 1));
    float m = v - c;

    float rf, gf, bf;
    if (h < 1.0/6)      { rf = c; gf = x; bf = 0; }
    else if (h < 2.0/6) { rf = x; gf = c; bf = 0; }
    else if (h < 3.0/6) { rf = 0; gf = c; bf = x; }
    else if (h < 4.0/6) { rf = 0; gf = x; bf = c; }
    else if (h < 5.0/6) { rf = x; gf = 0; bf = c; }
    else                 { rf = c; gf = 0; bf = x; }

    // RViz颜色空间补偿
    rf = pow(rf + m, gamma);
    gf = pow(gf + m, gamma);
    bf = pow(bf + m, gamma);

    r = static_cast<uint8_t>(rf * 255);
    g = static_cast<uint8_t>(gf * 255);
    b = static_cast<uint8_t>(bf * 255);
}


pcl::PointCloud<RGBPointType>::Ptr chosen_rgb_points(new pcl::PointCloud<RGBPointType>);
pcl::PointCloud<RGBPointType>::Ptr chosen_rgb_points2(new pcl::PointCloud<RGBPointType>);
pcl::PointCloud<RGBPointType>::Ptr chosen_rgb_points_compressed(new pcl::PointCloud<RGBPointType>);
pcl::PointCloud<RGBPointType>::Ptr rgb_points(new pcl::PointCloud<RGBPointType>);
pcl::PointCloud<RGBPointType>::Ptr rgb_points2(new pcl::PointCloud<RGBPointType>);
long double all_time = 0;// resample time
long double all_compressed_time = 0;//compressed time
long double all_mea = 0;//measurments num bit
long double all_mea_after_resample = 0 ;// MEA_AFTER_RESAMPLE  
long long int num= 0;
long long int points_num= 0;
long long int measurments = 0;
long long int all_measurments = 0;
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 
    int rq_0 = 0;
    int rq_1 = 0;
    int su_res = 0;
    int out_thr = 0;
    int out_count = 0;
    std::cout<<"feats_down_size is : "<< feats_down_size << std::endl;
    int points_max_num = max_points_all;
    float res_lo = thr_ / 2.00;
    std::cout<<"max_points_all : "<< points_max_num  << " quantizationLevel : " << quantizationLevel << std::endl;
    std::cout<<"len_d_ is : "<< len_d_ << std::endl;
    std::cout<<" float res_lo = thr_ / 2.00 : "<< res_lo << std::endl;
    int vect_len = len_d_* 2 * len_d_*2 * len_d_ * 2;// 64
    int vect_x = len_d_ *2 * len_d_ * 2; //16
    int vect_y = len_d_ * 2; // 4
    int vect_z = 1;
    int plane_num = 0;
    // int every_vect_max = static_cast<int>(max_points_all / len_d_) + 1 ;//4
    // quantized = std::max(0, std::min(quantized, (1 << bits) - 1));
    vector<int> count_len;
    std::cout<<"vect_len is : "<< vect_len << std::endl;
    count_len.resize(vect_len, 0);

    // 3D grid to store voxel information
    std::vector<std::vector<std::vector<IntervalInfo>>> interval_grid(
        len_d_ * 2,
        std::vector<std::vector<IntervalInfo>>(
            len_d_ * 2,
            std::vector<IntervalInfo>(len_d_ * 2)
        )
    );

    // 1D vector for mutexes
    int num_d_ = len_d_ * 2;
    std::vector<std::mutex> grid_mutexes(num_d_ * num_d_ * num_d_);
    std::atomic<int> fix(0);
    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        // omp_set_num_threads(1);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 
        // point x   point y   point z  q->tosome bits.
        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        // if(p_body.x() > 200. || p_body.x() < 200. || p_body.y() > 200. || p_body.y() < -200. || p_body.z() > 200. || p_body.z() < -200.)
        // {
        //     continue;
        // }
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;
        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        // 3D grid to store interval information
 

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
    
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            // std::cout<<"s is : "<< s << std::endl;
            // if (s > 0.9)
            if (s > 0.9)
            {   
                float pd2_abs = abs(pd2);
                float s_pd2_thr = 1.00000;
                float thr_s = thr_;
                if(use_resample)
                {
                    if(bit_disr == 1)
                    {
                        if(pd2_abs > 2 * thr_)//0.08 ouster 0.06 mid70->0.02 0.04  0.03->0.015
                        {
                            out_thr++;
                            point_selected_surf[i] = false;
                            continue;
                        }
                        thr_s = thr_;
                    }
                    else if(bit_disr == 2)
                    {
                        if(pd2_abs > 2 * thr_)//0.08 ouster 0.06 mid70->0.02 0.04  0.03->0.015
                        {
                            out_thr++;
                            point_selected_surf[i] = false;
                            continue;
                        }
                        if(pd2_abs > thr_)
                        {
                            thr_s = thr_; 
                        }
                        else
                        {
                            thr_s = 0.5 * thr_;
                        }
                    }
                    else
                    {
                        if(pd2_abs > thr_)//0.08 ouster 0.06 mid70->0.02 0.04  0.03->0.015
                        {
                            out_thr++;
                            point_selected_surf[i] = false;
                            continue;
                        }
                        thr_s = thr_;
                    }//
                    
                }
                s_pd2_thr = thr_s / thr_;
                int rx_ = static_cast<int>(s_pd2_thr * (pabcd(0) -0.001) / quantizationLevel);//0 1 2 3 
                int ry_ = static_cast<int>(s_pd2_thr * (pabcd(1) -0.001) / quantizationLevel);
                int rz_ = static_cast<int>(s_pd2_thr * (pabcd(2) -0.001) / quantizationLevel);
                int l = (rx_ + len_d_) * vect_x + (ry_ + len_d_) * vect_y + (rz_ + len_d_) * vect_z;
                // std::cout<<"0,1,2: "<< pabcd(0) << " " << pabcd(1) << " " << pabcd(2) << " over "<<std::endl;
                // std::cout<<"x,y,z: "<< rx_ + len_d_ << " " << ry_ + len_d_ << " " << rz_ + len_d_ << " over "<<std::endl;

                // Lock the mutex for this grid cell
                std::lock_guard<std::mutex> lock(grid_mutexes[l]);
                // Now it's safe to modify the IntervalInfo
                IntervalInfo &interval = interval_grid[rx_ + len_d_][ry_ + len_d_][rz_ + len_d_];
                interval.points.push_back({point_world.x, point_world.y, point_world.z, i});
                interval.normal = V3D(pabcd(0), pabcd(1), pabcd(2));
                plane_num++;
                if(!use_r)
                {
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                    point_selected_surf[i] = true;
                    
                }
                else
                {
                    normvec->points[i].x = (rx_ + 0.5) * quantizationLevel;
                    normvec->points[i].y = (ry_ + 0.5) * quantizationLevel;
                    normvec->points[i].z = (rz_ + 0.5) * quantizationLevel;
                    // only 距离
                    // //1 其实本质是2bit
                    // // 2. 1bit
                    // 
                    if((pd2 > 2.0 * thr_) || (pd2 < -2.0 * thr_) )
                    {
                        out_thr++;
                        point_selected_surf[i] = false;
                        continue;
                    }
                    else
                    {
                        if(bit_disr == 1)
                        {
                            if(pd2 > 0)
                            {
                                normvec->points[i].intensity = thr_;
                                res_last[i] = thr_;    
                                rq_1++;                 
                            }
                            else
                            {
                                rq_0++;
                                normvec->points[i].intensity = -thr_;
                                res_last[i] = thr_;   
                            }
                        }
                        else if(bit_disr == 2)
                        {
                            if(pd2 > 0)
                            {
                                if(pd2 > 2.0 * thr_)
                                {
                                    out_thr++;
                                    point_selected_surf[i] = false;
                                    continue;
                                }
                                if(pd2 > thr_)
                                {
                                    normvec->points[i].intensity = thr_;
                                    res_last[i] = thr_;  
                                }
                                else
                                {
                                    normvec->points[i].intensity = 0.5 * thr_;
                                    res_last[i] = 0.5 * thr_;  
                                }
                                // normvec->points[i].intensity = thr_;
                                // res_last[i] = thr_;    
                                rq_1++;                 
                            }
                            else
                            {
                                if(pd2 < (-2.0 * thr_))
                                {
                                    out_thr++;
                                    point_selected_surf[i] = false;
                                    continue;
                                }
                                if(pd2 < -thr_)
                                {
                                    normvec->points[i].intensity = -thr_;
                                    res_last[i] = thr_;  
                                }
                                else
                                {
                                    normvec->points[i].intensity = -0.5 * thr_;
                                    res_last[i] = 0.5 * thr_;  
                                }
                                rq_0++;
                                // normvec->points[i].intensity = -thr_;
                                // res_last[i] = thr_;   
                            }
                            point_selected_surf[i] = true;
                            su_res++;
                        }
                        else if(bit_disr==3)//
                        {
                            // float res_lo = thr_ / 2.00;
                            if(pd2 < 0)
                            {
                                int r_int = static_cast<int>((-pd2)/ res_lo);
                                float r_re = r_int * res_lo + res_lo * 0.5;
                                normvec->points[i].intensity = -r_re;
                                res_last[i] = r_re;
                            }
                            else
                            {
                                int r_int = static_cast<int>((pd2)/ res_lo);
                                float r_re = r_int * res_lo + res_lo * 0.5;
                                normvec->points[i].intensity = r_re;
                                res_last[i] = r_re;
                            }
                        }
                        else if(bit_disr==4)//
                        {
                            // float res_lo = thr_ / 4.00;
                            if(pd2 < 0)
                            {
                                int r_int = static_cast<int>((-pd2)/ res_lo);
                                float r_re = r_int * res_lo + res_lo * 0.5;
                                normvec->points[i].intensity = -r_re;
                                res_last[i] = r_re;
                            }
                            else
                            {
                                int r_int = static_cast<int>((pd2)/ res_lo);
                                float r_re = r_int * res_lo + res_lo * 0.5;
                                normvec->points[i].intensity = r_re;
                                res_last[i] = r_re;
                            }
                        }
                    }
                }
            }
        }


    }
    // std::cout<<"cal over"<<std::endl;
    // Second pass: Perform adaptive voxel downsampling for each interval
    std::vector<int> selected_indices;
    int down_voxel_nums = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    // std::cout<<" voxel size : ";
    //注释下面的东西，测时间
    auto start_time = std::chrono::steady_clock::now();
    //计算mea
    int temp_mea = 0;
    int q_p = bits_p;
    int q_v = static_cast<int>(1/quantizationLevel);
    int q_thr_= bit_disr + bits_p;//暂定2
    vector<int> count_beside;
    vector<int> count_after;
    // int num_d_ = len_d_ * 2;
    count_beside.resize(num_d_ * num_d_ * num_d_, 0);
    count_after.resize(num_d_ * num_d_ * num_d_, 0);
    if(use_resample)
    {
        // std::cout<<"distance_factor dis_scale : "<<dis_scale << std::endl;
        std::fill(point_selected_surf, point_selected_surf+100000, false);
        // 清空点云并预留空间
        rgb_points->clear();
        chosen_rgb_points->clear();
        chosen_rgb_points_compressed->clear();
        rgb_points->reserve(selected_indices.size());
        const int color_step = 255 / (2 * len_d_); // 根据网格维度生成颜色梯度
        for (int rx = 0; rx < 2 * len_d_; rx++)
        {
            for (int ry = 0; ry < 2 * len_d_; ry++)
            {
                for (int rz = 0; rz < 2 * len_d_; rz++)
                {
                    // Lock the mutex for this grid cell
                    // Calculate 1D index for mutex
                    size_t mutex_index = (rx * num_d_ * num_d_) + (ry * num_d_) + rz;
                    // Lock the mutex for this grid cell
                    std::lock_guard<std::mutex> lock(grid_mutexes[mutex_index]);
                    IntervalInfo &interval = interval_grid[rx][ry][rz];

                    if (interval.points.empty()) continue;

                    // 转换HSV到RGB
                    // int r, g, b;
                    // ============== 修改颜色生成部分 ==============
                    // 基础颜色步长（len_d=2时生成4级：0,85,170,255）
                    const int step = 255 / (2 * len_d_ - 1); 
                    int r = rx * step;
                    int g = ry * step;
                    int b = rz * step;
                    
                    // 增强颜色区分度（保持相邻体素颜色差异>64）
                    r = (r + (ry % 2)*40) % 256;
                    g = (g + (rz % 3)*30) % 256;
                    b = (b + (rx % 4)*20) % 256;
                    for (const auto &p : interval.points) {
                        // 原始点坐标
                        const auto& src_pt = feats_down_body->points[p.original_index];
                        // 创建带颜色的点
                        RGBPointType colored_pt;
                        colored_pt.x = src_pt.x;
                        colored_pt.y = src_pt.y;
                        colored_pt.z = src_pt.z;
                        colored_pt.r = r;
                        colored_pt.g = g;
                        colored_pt.b = b;
                        // 线程安全地添加点
                        #pragma omp critical
                        {
                            rgb_points->push_back(colored_pt);
                        }
                    }
                    interval_grid[rx][ry][rz].r = r;
                    interval_grid[rx][ry][rz].g = g;
                    interval_grid[rx][ry][rz].b = b;
                    // Calculate adaptive voxel size based on point count and distance
                    double avg_distance = 0;
                    for (const auto &p : interval.points) {
                        int id_ori = p.original_index;
                        avg_distance += std::sqrt(feats_down_body->points[id_ori].x * feats_down_body->points[id_ori].x 
                                                + feats_down_body->points[id_ori].y * feats_down_body->points[id_ori].y 
                                                + feats_down_body->points[id_ori].z * feats_down_body->points[id_ori].z);
                    }
                    avg_distance /= interval.points.size();
                    count_beside.at(mutex_index) = interval.points.size();
                    // Adjust these factors as needed
                    double base_voxel_size = filter_size_surf_min;  // Base voxel size in meters
                    double distance_factor = dis_scale; // How much to increase voxel size per meter of distance
                    double count_factor = 0.005;   // How much to decrease voxel size per additional point
                    
                    double voxel_size = base_voxel_size + distance_factor * avg_distance;
                    // std::cout<<" "<< voxel_size;
                    voxel_size = std::max(voxel_size, filter_size_surf_min * 2); // Minimum voxel size

                    // Perform voxel downsampling
                    std::unordered_map<V3I, std::vector<Point3D>, V3IHash> voxels;
                    for (const auto &p : interval.points) {
                        V3I voxel_coord(
                            static_cast<int>(p.x / voxel_size),
                            static_cast<int>(p.y / voxel_size),
                            static_cast<int>(p.z / voxel_size)
                        );
                        voxels[voxel_coord].push_back(p);
                    }
                    int num_k = 0;
                    // Select one point from each voxel
                    for (const auto &voxel : voxels) {
                        if (!voxel.second.empty()) {
                            // Choose a random point from the voxel
                            num_k++;
                            // mea
                            std::uniform_int_distribution<> dis(0, voxel.second.size() - 1);
                            int random_index = dis(gen);
                            selected_indices.push_back(voxel.second[random_index].original_index);
                            down_voxel_nums++;
                            point_selected_surf[voxel.second[random_index].original_index] = true;
                            RGBPointType colored_pt_select, colored_pt_select_q;
                            colored_pt_select.x = voxel.second[random_index].x;
                            colored_pt_select.y = voxel.second[random_index].y;
                            colored_pt_select.z = voxel.second[random_index].z;
                            colored_pt_select.r = r;
                            colored_pt_select.g = g;
                            colored_pt_select.b = b;
                            //
                            colored_pt_select_q.x = feats_down_body->points[voxel.second[random_index].original_index].x;
                            colored_pt_select_q.y = feats_down_body->points[voxel.second[random_index].original_index].y;
                            colored_pt_select_q.z = feats_down_body->points[voxel.second[random_index].original_index].z;
                            colored_pt_select_q.r = r;
                            colored_pt_select_q.g = g;
                            colored_pt_select_q.b = b;
                            
                            #pragma omp critical
                            {
                                chosen_rgb_points->push_back(colored_pt_select);
                                chosen_rgb_points_compressed->push_back(colored_pt_select_q);
                            }
                        }
                    }
                    count_after.at(mutex_index)=num_k;
                    temp_mea = temp_mea + num_k * q_thr_ + q_v;
                }
            }
        }
    }    
    auto end_time = std::chrono::steady_clock::now();
    // 计算时间间隔
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    int a = duration.count();
    std::cout<<"q_thr_ : "<< q_thr_ << " = "<< bit_disr << "  + " << bits_p << std::endl;
    num++;
    all_time += static_cast<double>(a);
    double avagetime = all_time / num;
    long int mea_ori = 28 * 32 * down_voxel_nums;
    // std::cout << "耗时: " <<a << " " <<duration.count() << " 微秒" << std::endl;
    std::cout << "avage :"<< avagetime << " after_temp_mea : " << temp_mea << " #beside: "<< mea_ori <<
    " down_voxel_nums : "
    << down_voxel_nums <<" #beside : "<< su_res <<" \n all plane_num : " << plane_num << std::endl;
    // std::cout<<" over\n ";
    // 保存位姿到指定的文件
    std::string save_txt = save_dir + "/res_nums.txt";
    std::ofstream save_file;
    save_file.open(save_txt, std::ios::app | std::ios::out); // 以追加模式打开文件

    if (!save_file.is_open()) {
        ROS_ERROR("无法打开文件: %s", save_txt.c_str());
        return;
    }

    // 设置输出精度
    // temp_mea 单帧总测量位数
    // mea_ori 原始总测量Byte
    // down_voxel_nums 用的点总数
    // su_res通过的优化
    // plane_num总点数
    save_file << std::fixed << std::setprecision(6);
    save_file << avagetime << " "
                << temp_mea <<" "
                << mea_ori << " "
                << down_voxel_nums << " " 
                << su_res <<" "
                << plane_num << "\n";
    save_file.close();
                // << out_thr << " "
                // << out_count << " "
                // << su_res << " "
                // << rq_0 << " "
                // << rq_1 << " "
                // << thr_;
    //分布
    std::string save_beside_after_txt = save_dir + "/beside_after.txt";
    std::ofstream save_beside_after;
    save_beside_after.open(save_beside_after_txt, std::ios::app | std::ios::out); // 以追加模式打开文件
    if (!save_beside_after.is_open()) {
        ROS_ERROR("无法打开文件: %s", save_beside_after_txt.c_str());
        return;
    }
    for (int i = 0; i < count_beside.size();i++)
    {
        save_beside_after << count_beside[i] <<" "<<count_after[i] << " " ;
    }
    save_beside_after << "\n" ;
    save_beside_after.close();
    std::cout<<" beside_after[0] : "<< count_beside[0] << " " << count_after[0] << 
    out_count << " down_voxel_nums : " << down_voxel_nums<< std::endl;
    // std::cout<<" sucess: "<< su_res << std::endl;
    // std::cout<< "rq_0 / rq_1 : "<< rq_0 << " " << rq_1 << " thr is" << thr_ << std::endl;
    effct_feat_num = 0;
    int k = 0;

    if(!use_r)
    {
        laserCloudWorld_1->clear();
        laserCloudWorld_1->points.resize(feats_down_size);
        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                laserCloudWorld_1->points[effct_feat_num] = feats_down_body->points[i];

                total_residual += res_last[i];
                effct_feat_num ++;
                k++;
            }
        }
    }
    else
    {
        laserCloudWorld_1->clear();
        laserCloudWorld_1->points.resize(feats_down_size);
        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                laserCloudWorld_1->points[effct_feat_num] = feats_down_body->points[i];

                total_residual += res_last[i];
                effct_feat_num ++;
                k++;
            }
        }
    }
    // else
    // {
    //     laserCloudWorld->points.resize(selected_indices.size());
    //     for (int i = 0; i < selected_indices.size(); i++)
    //     {
    //         int num = selected_indices[i];
    //         if (point_selected_surf[selected_indices[i]])
    //         {
    //             laserCloudOri->points[effct_feat_num] = feats_down_body->points[num];
    //             corr_normvect->points[effct_feat_num] = normvec->points[num];
    //             laserCloudWorld->points[effct_feat_num] = feats_down_body->points[num];

    //             total_residual += res_last[num];
    //             effct_feat_num ++;
    //             k++;
    //         }
    //     }
    // }
    

    std::cout<<"effct_feat_num : "<< effct_feat_num << " sucess: "<< su_res << std::endl;
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);
    std::cout<<"#########################time to const H"<<std::endl;
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        float x_q, y_q, z_q;
        V3D point_this_be;
        if(use_r)
        {
            if(i % 1000 ==0) std::cout<<"use ris "<< use_r << std::endl;
            if(float_or_fixed)//float
            {
                // 浮点量化
                // std::cout << "\nFloating-point quantization:" << std::endl;
                x_q = pointCompressor.quantizeFloat(laser_p.x);
                y_q = pointCompressor.quantizeFloat(laser_p.y);
                z_q = pointCompressor.quantizeFloat(laser_p.z);
            }
            else// fixed
            {//compressed time
                auto start_time1 = std::chrono::steady_clock::now();
                x_q = pointCompressor.quantizeFixedRange(laser_p.x);
                y_q = pointCompressor.quantizeFixedRange(laser_p.y);
                z_q = pointCompressor.quantizeFixedRange(laser_p.z);
                auto end_time1 = std::chrono::steady_clock::now();
                // 计算时间间隔
                auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end_time1- start_time1);
                int a1 = duration1.count();
                points_num++;
                all_compressed_time += static_cast<double>(a1);
            }
            point_this_be<<x_q, y_q, z_q;
        }
        else
        {
            point_this_be<< laser_p.x, laser_p.y, laser_p.z;
        }
        // if(laser_p.x > 200 || laser_p.x < 200 || laser_p.y > 200 || laser_p.y < -200 || laser_p.z > 200 || laser_p.z < -200)
        // {
        //     if(float_or_fixed)//float
        //     {
        //         // 浮点量化
        //         // std::cout << "\nFloating-point quantization:" << std::endl;
        //         x_q = pointCompressor.quantizeFloat(laser_p.x);
        //         y_q = pointCompressor.quantizeFloat(laser_p.y);
        //         z_q = pointCompressor.quantizeFloat(laser_p.z);
        //     }
        //     else// fixed
        //     {
        //         x_q = pointCompressor.quantizeFixedRange(laser_p.x);
        //         y_q = pointCompressor.quantizeFixedRange(laser_p.y);
        //         z_q = pointCompressor.quantizeFixedRange(laser_p.z);
        //     }
        // }
        // else
        // {
        //     continue;
        // }
        // V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        // norm_vec = 0.868 * norm_vec;
        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) <<  norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    std::cout<<"###################################3const H over"<<std::endl;
    solve_time += omp_get_wtime() - solve_start_;
    double avage_compressed = all_compressed_time / num;
    std::cout<<"aveage_compressed_time " << avage_compressed << std::endl;
}
long int num_save = 0;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 2);
    
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<bool>("float_or_fixed", float_or_fixed , true);
    nh.param<double>("dis_scale", dis_scale , 0.05);
    // if true processPointCloudFloat
    // compressor.processPointCloudFixedRange(original_points);
    // compressor.processPointCloudFloat(original_points);
    nh.param<bool>("use_r", use_r, false);
    nh.param<bool>("use_zr", use_zr, false);
    nh.param<bool>("save_pcd", save_pcd, false);
    nh.param<bool>("use_resample",use_resample , false);
    nh.param<int>("bits_r", bits_r , 1);
    nh.param<int>("bits_p", bits_p , 6);
    nh.param<int>("bit_disr", bit_disr , 2);
    nh.param<int>("max_points_all",max_points_all,500);
    nh.param<float>("thr_", thr_, 0.1);
    nh.param<float>("thr_2", thr_2, 0.2);
    nh.param<float>("thr_3", thr_3, 0.3);
    nh.param<float>("thr_4", thr_4, 0.4);
    nh.param<float>("quantizationLevel", quantizationLevel , 0.01);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<string>("save_dir", save_dir, "/home/lbd/slam/Lio/fastlio_myq/src/MCD/ntu/ntu_day_01/15/");
    len_d_ = static_cast<int>(1 / quantizationLevel); // 3 * len_dir_
    every_vect_max = static_cast<int>(max_points_all / len_d_) + 1 ;//4
    // int vect_len = 64;
    // count_len.resize(vect_len, 0);
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    cout<<"using float_or_fixed" << float_or_fixed << std::endl;
    cout<<"using points bits as : "<< bits_p << std::endl;
    cout<<"max points is : "<< max_points_all << std::endl;
    cout<<"len_d_ is : "<< len_d_ << std::endl;
    cout<<"every_vect_max is : "<< every_vect_max << std::endl;
    cout<<"filter_size_surf : "<< filter_size_surf_min <<endl;
    cout<<"using use_resample : "<< use_resample << endl;
    cout<<"bit_disr is : "<< bit_disr << std::endl;
    cout<<"thr is : "<< thr_ << std::endl;
    cout<<"save_pcd is : "<< save_pcd << std::endl; 
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";
    pointCompressor.set_bits(bits_p);
    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);

    ros::Publisher pub_undistort_cloud = nh.advertise<sensor_msgs::PointCloud2>
            ("/undistort_cloud", 100);
    ros::Publisher pubPath_          = nh.advertise<nav_msgs::Odometry> 
            ("/path_", 10);    
    rgb_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
            ("/color_points", 100000);     
    rgb_chosen_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
            ("/color_points_chosen", 100000);   
    rgb_chosen_compressed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
            ("/color_points_chosen_compressed", 100000);   
    double current_time; 
    cout<<"thr : "<< thr_ <<std::endl;      
    cout<<"vec quantizationLevel: "<< quantizationLevel << std::endl; 
    cout<<"dis_scale : "<< dis_scale << std::endl;
    cout<<"use_resample: "<< use_resample << std::endl;
    createDirectoryIfNotExists(save_dir);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        current_time = time_buffer.front();
        if(sync_packages(Measures)) 
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();
            if(use_zr)
            {
                for(int k = 0; k < Measures.lidar->points.size(); k++)
                {
                    Measures.lidar->points[k].x = pointCompressor.quantizeFixedRange(Measures.lidar->points[k].x);
                    Measures.lidar->points[k].y = pointCompressor.quantizeFixedRange(Measures.lidar->points[k].y);
                    Measures.lidar->points[k].z = pointCompressor.quantizeFixedRange(Measures.lidar->points[k].z);
                }
            }
            p_imu->Process(Measures, kf, feats_undistort);// change to 时间差值
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time); // H * = sqrt(2/sqrt(2*pi))
            // kf.update_iterated_dyn_share_modified_rq(LASER_POINT_COV, solve_H_time); // rq with sqrt_H 
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();
            ///////////////////////////////////////
            //color 
            // 发布带颜色的点云
            //所有可能参与优化的点
            num_save++;
            if (rgb_points->points.size()!=0) {
                sensor_msgs::PointCloud2 output;
                rgb_points2->clear();
                rgb_points2->resize(rgb_points->points.size());
                for (int i = 0; i < rgb_points->points.size(); i++)
                {
                    rRGBpointBodyToWorld(&rgb_points->points[i], \
                                    &rgb_points2->points[i]);
                    if(i==100)
                    {
                        cout<<"r g b"<<rgb_points2->points[i].r << " " << rgb_points2->points[i].g << " " <<rgb_points2->points[i].b<<endl;
                    }
                    
                }
                if(save_pcd)
                {
                    std::ostringstream oss;
                    // 设置4位固定宽度，不足补零 (0000-9999)
                    oss << std::setw(4) << std::setfill('0') << num_save; 
                    std::string filename = save_dir +'/' + oss.str() + "_all_plane.pcd";
                    cout<<"rgb_points2.size:" << rgb_points2->points.size() << std::endl;
                    rgb_points2->width = rgb_points2->size();
                    rgb_points2->height = 1;
                    pcl::io::savePCDFileBinary(filename, *rgb_points2);
                    std::cout << "Saved: " << filename << std::endl;
                }
                pcl::toROSMsg(*rgb_points2, output);
                output.header.stamp = ros::Time().fromSec(lidar_end_time);
                output.header.frame_id = "camera_init"; // 设置正确的坐标系
                rgb_cloud_pub.publish(output);
                rgb_points2->clear();
            }
            ///////////////////////////////////////
            //选中的的点
            //参与优化的点
            if (chosen_rgb_points->points.size()!=0) 
            {
                sensor_msgs::PointCloud2 output;
                chosen_rgb_points2->clear();
                chosen_rgb_points2->resize(chosen_rgb_points->points.size());
                for (int i = 0; i < chosen_rgb_points->points.size(); i++)
                {
                    // rRGBpointBodyToWorld(&chosen_rgb_points->points[i], \
                    //                 &chosen_rgb_points2->points[i]);
                    // chosen_rgb_points2->points.push_back(chosen_rgb_points->points[i]);
                    // if(i%50==0)
                    // {
                    //     cout<<"x1 y1 z1 "<<chosen_rgb_points->points[i].x << " " << chosen_rgb_points->points[i].y << " " <<chosen_rgb_points->points[i].z<<endl;
                    //     cout<<"x2 y2 z2 "<<chosen_rgb_points2->points[i].x << " " << chosen_rgb_points2->points[i].y << " " <<chosen_rgb_points2->points[i].z<<endl;
                    // }
                }
                if(save_pcd)
                {
                    std::ostringstream oss;
                    // 设置4位固定宽度，不足补零 (0000-9999)
                    oss << std::setw(4) << std::setfill('0') << num_save; 
                    std::string filename = save_dir +'/' + oss.str() + "_chosen.pcd";
                    cout<<"chosen_rgb_points2.size:" << chosen_rgb_points->points.size() << std::endl;
                    chosen_rgb_points->width = chosen_rgb_points2->size();
                    chosen_rgb_points->height = 1;
                    pcl::io::savePCDFileBinary(filename, *chosen_rgb_points);
                    std::cout << "Saved: " << filename << std::endl;
                }
                pcl::toROSMsg(*chosen_rgb_points, output);
                output.header.stamp = ros::Time().fromSec(lidar_end_time);
                output.header.frame_id = "camera_init"; // 设置正确的坐标系
                rgb_chosen_cloud_pub.publish(output);
                // chosen_rgb_points2->clear();
            }  
            double all_sqrt_error = 0.;
            int avaerage = 0;
            //量化后的点
            //参与优化量化后的点
            if(chosen_rgb_points_compressed->points.size()!=0)
            {
                sensor_msgs::PointCloud2 output;
                chosen_rgb_points2->clear();
                chosen_rgb_points2->resize(chosen_rgb_points->points.size());
                for (int i = 0; i < chosen_rgb_points->points.size(); i++)
                {
                    avaerage++;
                    float x_q = pointCompressor.quantizeFixedRange(chosen_rgb_points_compressed->points[i].x);
                    float y_q = pointCompressor.quantizeFixedRange(chosen_rgb_points_compressed->points[i].y);
                    float z_q = pointCompressor.quantizeFixedRange(chosen_rgb_points_compressed->points[i].z);
                    all_sqrt_error += sqrt((x_q - chosen_rgb_points_compressed->points[i].x) * (x_q - chosen_rgb_points_compressed->points[i].x)
                    +(y_q - chosen_rgb_points_compressed->points[i].y) * (y_q - chosen_rgb_points_compressed->points[i].y) 
                    +(z_q - chosen_rgb_points_compressed->points[i].z) * (z_q - chosen_rgb_points_compressed->points[i].z));
                    chosen_rgb_points_compressed->points[i].x = x_q;
                    chosen_rgb_points_compressed->points[i].y = y_q;
                    chosen_rgb_points_compressed->points[i].z = z_q;
                    rRGBpointBodyToWorld(&chosen_rgb_points_compressed->points[i], \
                                    &chosen_rgb_points2->points[i]);
                    // chosen_rgb_points2->points.push_back(chosen_rgb_points->points[i]);
                    // if(i%50==0)
                    // {
                    //     cout<<"x1 y1 z1 "<<chosen_rgb_points->points[i].x << " " << chosen_rgb_points->points[i].y << " " <<chosen_rgb_points->points[i].z<<endl;
                    //     cout<<"x2 y2 z2 "<<chosen_rgb_points2->points[i].x << " " << chosen_rgb_points2->points[i].y << " " <<chosen_rgb_points2->points[i].z<<endl;
                    // }
                }
                if(save_pcd)
                {
                    std::ostringstream oss;
                    // 设置4位固定宽度，不足补零 (0000-9999)
                    oss << std::setw(4) << std::setfill('0') << num_save; 
                    std::string filename = save_dir +'/' + oss.str() + "_chosencompressed.pcd";
                    cout<<"chosen_rgb_points_compressed.size:" << chosen_rgb_points_compressed->points.size() << std::endl;
                    chosen_rgb_points2->width = chosen_rgb_points_compressed->size();
                    chosen_rgb_points2->height = 1;
                    pcl::io::savePCDFileBinary(filename, *chosen_rgb_points2);
                    std::cout << "Saved: " << filename << std::endl;
                }
                all_sqrt_error = all_sqrt_error / (static_cast<double>(avaerage));
                cout<<"error :" << all_sqrt_error<<endl;
                pcl::toROSMsg(*chosen_rgb_points2, output);
                output.header.stamp = ros::Time().fromSec(lidar_end_time);
                output.header.frame_id = "camera_init"; // 设置正确的坐标系
                rgb_chosen_compressed_cloud_pub.publish(output);
                chosen_rgb_points2->clear();
            }          
            // ori 
            int size = laserCloudWorld_1->points.size();
            PointCloudXYZI::Ptr laserCloudWorld2( \
                            new PointCloudXYZI(size, 1));  
            sensor_msgs::PointCloud2 laserCloudusing_fortest;  
            for (int i = 0; i < size; i++)
            {
                RGBpointBodyToWorld(&laserCloudWorld_1->points[i], \
                                    &laserCloudWorld2->points[i]);
            }
            pcl::toROSMsg(*laserCloudWorld2, laserCloudusing_fortest);
            laserCloudusing_fortest.header.stamp = ros::Time().fromSec(lidar_end_time);
            laserCloudusing_fortest.header.frame_id = "camera_init";
            pubLaserCloudEffect.publish(laserCloudusing_fortest);
            laserCloudWorld_1->points.clear();
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            nav_msgs::Odometry current_pose;
            current_pose.header.stamp          = ros::Time().fromSec(current_time);
            current_pose.header.frame_id       = "camera_init";


            // #ifdef USE_IKFOM
            // current_pose.pose.pose.position.x = state_point.pos(0);
            // current_pose.pose.pose.position.y = state_point.pos(1);
            // current_pose.pose.pose.position.z = state_point.pos(2);
            // #else
            // current_pose.pose.pose.position.x = state.pos_end(0);
            // current_pose.pose.pose.position.y = state.pos_end(1);
            // current_pose.pose.pose.position.z = state.pos_end(2);
            // #endif        
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            current_pose.pose.pose.position.x = pos_lid(0);
            current_pose.pose.pose.position.y = pos_lid(1);
            current_pose.pose.pose.position.z = pos_lid(2);        
            
            current_pose.pose.pose.orientation.x = geoQuat.x;
            current_pose.pose.pose.orientation.y = geoQuat.y;
            current_pose.pose.pose.orientation.z = geoQuat.z;
            current_pose.pose.pose.orientation.w = geoQuat.w;
            Eigen::Vector3d position_eigen(current_pose.pose.pose.position.x,
                               current_pose.pose.pose.position.y,
                               current_pose.pose.pose.position.z);
            // 创建 Eigen::Quaterniond 对象
            Eigen::Quaterniond orientation_eigen(
                current_pose.pose.pose.orientation.w,
                current_pose.pose.pose.orientation.x,
                current_pose.pose.pose.orientation.y,
                current_pose.pose.pose.orientation.z);

            // // 然后将其传入 appendPoseToTUM 函数
            // appendPoseToTUM("/home/lbd/slam_dataset/meta_edu/fastlio/src/FAST_LIO-main/src/balm.txt", 
            //                 current_time, 
            //                 position_eigen, 
            //                 orientation_eigen);
            //             // 发布路径

            pubPath_.publish(current_pose);


            sensor_msgs::PointCloud2 cloud_out;
            // std::cout<<"feats_undistort size:"<<feats_undistort->size()<<std::endl;
            pcl::toROSMsg(*feats_undistort, cloud_out);
    
            cloud_out.header.stamp = ros::Time().fromSec(current_time);
            cloud_out.header.frame_id = "camera_init";
            pub_undistort_cloud.publish(cloud_out);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
