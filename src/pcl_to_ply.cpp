#include <fstream>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    union {
        std::uint32_t t; // time since beginning of scan in nanoseconds
        float time; // time since beginning of scan in seconds
        double timestamp; // absolute timestamp in seconds
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (float, time, time)
    (double, timestamp, timestamp))

struct pcl_to_ply: public rclcpp::Node {
    pcl_to_ply(): Node("pcl_to_ply") {
        _sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 
            100,
            std::bind(&pcl_to_ply::callback_points, this, std::placeholders::_1)
        );

        std::ios::sync_with_stdio(false);
        _ofs.open("pointcloud.ply", std::ios::binary);
        _ofs.rdbuf()->pubsetbuf(_io_buffer.data(), _io_buffer.size());
        _ofs << "ply\n";
        _ofs << "format ascii 1.0\n";
        // _ofs << "format binary_little_endian 1.0\n";
        _ofs << "element vertex                         \n";
        _ofs << "property float x\n";
        _ofs << "property float y\n";
        _ofs << "property float z\n";
        _ofs << "end_header\n";
    }
    ~pcl_to_ply() {
        // write vertex count
        _ofs.seekp(36, std::ios::beg);
        // _ofs.seekp(51, std::ios::beg);
        _ofs << _point_count;
        _ofs.close();
    }

    void callback_points(const sensor_msgs::msg::PointCloud2& msg) {
        // extract pointcloud from message
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(msg, pointcloud);
        std::cout << "Received " << pointcloud.points.size() << " points" << std::endl;
        for (const auto& point: pointcloud.points) {
            _ofs << point.x << ' ' << point.y << ' ' << point.z << '\n';
        }
        _point_count += pointcloud.points.size();
    }

    std::ofstream _ofs;
    std::size_t _point_count;
    std::array<char, 256*1024> _io_buffer;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_pcl;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcl_to_ply>());
    rclcpp::shutdown();
    return 0;
}