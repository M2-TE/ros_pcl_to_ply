#include <fstream>
#include <sstream>
#include <iomanip>

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

class pcl_to_ply: public rclcpp::Node 
{
public:   
    pcl_to_ply(): Node("pcl_to_ply") 
    {

        _frame_count        = 0;
        _local_map_size     = 1000;
        _local_map_points   = 0;
        _local_map_id       = 0;

        _sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 
            100,
            std::bind(&pcl_to_ply::callback_frame, this, std::placeholders::_1)
        );
    }

private:
    void callback_frame(const sensor_msgs::msg::PointCloud2& msg) 
    {
        RCLCPP_INFO(get_logger(), "Receiving frame %zu from %zu for chunk %zu",  _frame_count, _local_map_size, _local_map_id);
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(msg, pointcloud);
        _frame_count++;
        _local_map_points += pointcloud.points.size();
        for(const auto& point: pointcloud.points)
        {
            _points.push_back(point);
        }
        if(_frame_count > _local_map_size)
        {
            saveLocalMap();
        }
    }

    void saveLocalMap()
    {
        // Generate file name and output stream for current chunk
        std::stringstream sstr;
        sstr << "chunk" << std::setfill('0') << std::setw(5) << _local_map_id << ".ply";
        std::ofstream out;
        out.open(sstr.str().c_str(), std::ios::binary);

        RCLCPP_INFO(get_logger(), "Writing chunk %zu to file %s", _local_map_id, sstr.str().c_str());

        // Write PLY header
        out << "ply" << std::endl;
        out << "format binary_little_endian 1.0" << std::endl;
        out << "element vertex " << _local_map_points << std::endl;
        out << "property float x" << std::endl;
        out << "property float y" << std::endl;
        out << "property float z" << std::endl;
        out << "property float intensity" << std::endl;
        out << "end_header" << std::endl;

        // Write binary blob with point cloud data (should be buffered in next
        // version but currently performance seems to be sufficient 
        for(const auto& point: _points)
        {
            out.write(reinterpret_cast<const char *>(&point.x), sizeof(float));
            out.write(reinterpret_cast<const char *>(&point.y), sizeof(float));
            out.write(reinterpret_cast<const char *>(&point.z), sizeof(float));
            out.write(reinterpret_cast<const char *>(&point.intensity), sizeof(float));
        }
        
        // Reset counters
        _points.clear();
        _local_map_points = 0;
        _frame_count = 0;
        _local_map_id++;
    }

    /// Subscription to descewed point clouds
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_pcl;

    /// @brief  Buffer for current local map
    std::vector<Point> _points;

    /// @brief  Number of frames in current local map
    size_t _frame_count;

    /// @brief  Number of frames per local map
    size_t _local_map_size;

    /// @brief  Number of points in current local map
    size_t _local_map_points;

    /// @brief  Map id (chunk number)
    size_t _local_map_id;
    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcl_to_ply>());
    rclcpp::shutdown();
    return 0;
}