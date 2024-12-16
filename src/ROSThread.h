#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/timer.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/magnetic_field.hpp>
#include "novatel_gps_msgs/msg/inspva.hpp"

#include <rosgraph_msgs/msg/clock.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

#include "std_srvs/srv/set_bool.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "helipr_file_player/color.h"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>

#include "helipr_file_player/datathread.h"
#include <sys/types.h>
#include <algorithm>
#include <iterator>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sstream>

#include <livox_ros_driver2/msg/custom_msg.hpp>

using namespace std;
using namespace cv;

struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint16_t ring;
    std::uint16_t ambient;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity) 
    (std::uint16_t, ring, ring) (std::uint16_t, ambient, ambient)
)

struct AevaPointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY
    float reflectivity;
    float velocity;
    std::int32_t time_offset_ns;
    std::uint8_t line_index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(AevaPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, reflectivity, reflectivity) (float, velocity, velocity) 
    (std::int32_t, time_offset_ns, time_offset_ns) (std::uint8_t, line_index, line_index)
)

// struct LivoxPointXYZI
// {
//     PCL_ADD_POINT4D;
//     std::uint8_t reflectivity;
//     std::uint8_t tag;
//     std::uint8_t line;
//     std::uint32_t offset_time;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZI,
//     (float, x, x) (float, y, y) (float, z, z) (std::uint8_t, reflectivity, reflectivity)
//     (std::uint8_t, tag, tag) (std::uint32_t, line, line) (std::uint32_t, offset_time, offset_time)
// )

struct ContinentalPointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY
    float v; 
    float r;
    std::int8_t RCS; 
    float azimuth;
    float elevation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(ContinentalPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, v, v)
    (float, r, r) (std::int8_t, RCS, RCS)
    (float, azimuth, azimuth) (float, elevation, elevation) 
)

struct ContinentalXYZV
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY
    float vx; 
    float vy;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(ContinentalXYZV,
    (float, x, x) (float, y, y) (float, z, z)
    (float, vx, vx) (float, vy, vy)
)

struct Point3D {
    float x;
    float y;
    float z;
    Point3D(float x_, float y_, float z_)
      : x(x_), y(y_), z(z_) {}
};

using pc_type = PointXYZIRT;
using pc_type_o = OusterPointXYZIRT;
// using pc_type_l = LivoxPointXYZI;
using pc_type_a = AevaPointXYZIRT;

// using pc_type_c = ContinentalPointXYZIRT;
// using pc_type_co = ContinentalXYZV;

class ROSThread : public QThread
{
    Q_OBJECT

public:
    int64_t initial_data_stamp_;
    int64_t last_data_stamp_;

    int stamp_show_count_;

    bool auto_start_flag_;
    bool play_flag_;
    bool pause_flag_;
    bool save_flag_;
    bool loop_flag_;
    bool stop_skip_flag_;
    double play_rate_;
    bool process_flag_;
    int bag_idx_;

    rosbag2_cpp::Writer bag_;
    rosbag2_storage::StorageOptions storage_options;
    std::string data_folder_path_;

    QMutex *mutex_;
    std::mutex bag_mutex_;
    rclcpp::Node::SharedPtr node_;



    explicit ROSThread(QObject *parent = nullptr, QMutex *th_mutex = nullptr);
    ~ROSThread();
    void ros_initialize(rclcpp::Node::SharedPtr node);
    void run();
    void SaveRosbag();
    void Ready();
    void ResetProcessStamp(int position);
    void FilePlayerStart(const std_msgs::msg::Bool::SharedPtr msg);
    void FilePlayerStop(const std_msgs::msg::Bool::SharedPtr msg);
    // void TimerCallback(const rclcpp::TimerEvent& event);
    void TimerCallback();
    int GetDirList(const std::string dir, std::vector<std::string> &files);

signals:
    void StampShow(quint64 stamp);
    void StartSignal();

private:
    
    int search_bound_;
    float minimum;

    int imu_data_version_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

    rclcpp::Publisher<novatel_gps_msgs::msg::Inspva>::SharedPtr inspva_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aeva_pub_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr avia_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ouster_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_pub_;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;


    rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic tasks

    void DataStampThread();
    void InspvaThread();
    void ImuThread();
    void AevaThread();
    void AviaThread();
    void OusterThread();
    void VelodyneThread();


    std::vector<std::string> aeva_file_list_;
    std::vector<std::string> avia_file_list_;
    std::vector<std::string> ouster_file_list_;
    std::vector<std::string> velodyne_file_list_;

    int64_t prev_clock_stamp_;
    int64_t processed_stamp_;
    int64_t pre_timer_stamp_;
    bool reset_process_stamp_flag_;


    std::map<int64_t, int64_t> stop_period_;

    std::multimap<int64_t, std::string>                 data_stamp_;
    std::map<int64_t, novatel_gps_msgs::msg::Inspva>    inspva_data_;
    std::map<int64_t, sensor_msgs::msg::Imu>            imu_data_;
    std::map<int64_t, sensor_msgs::msg::MagneticField>  mag_data_;

    DataThread<int64_t> data_stamp_thread_;
    DataThread<int64_t> imu_thread_;
    DataThread<int64_t> inspva_thread_;
    DataThread<int64_t> aeva_thread_;
    DataThread<int64_t> avia_thread_;
    DataThread<int64_t> ouster_thread_;
    DataThread<int64_t> velodyne_thread_;


    std::pair<std::string, sensor_msgs::msg::PointCloud2> aeva_next_;
    std::pair<std::string, livox_ros_driver2::msg::CustomMsg> avia_next_;
    std::pair<std::string, sensor_msgs::msg::PointCloud2> ouster_next_;
    std::pair<std::string, sensor_msgs::msg::PointCloud2> velodyne_next_;

};

#endif // VIEWER_ROS_H
