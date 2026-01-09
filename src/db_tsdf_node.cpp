/*
 * TSDFNode: ROS 2 node for LiDAR->TDF mapping.
 * Subscribes to PointCloud2, looks up the transform via tf2_ros::Buffer,
 * filters, transforms, and integrates the cloud into the TSDF grid.
 * Publishes the filtered, global-frame cloud for visualization.
 */

#include <vector>
#include <thread>
#include <chrono>
#include <memory>      
#include <iomanip>
#include <sstream>
#include <mutex>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp> 

// TF2
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// PCL
#include <pcl/point_types.h> 
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/common/transforms.h> 

// DB-TSDF
#include <db_tsdf/tsdf3d_16.hpp>
#include <db_tsdf/grid_16.hpp>
   
class TSDFNode : public rclcpp::Node
{
public:
    TSDFNode(const std::string &node_name)
        : Node(node_name)
    {
        // Parameters
        m_inCloudTopic      = this->declare_parameter<std::string>("in_cloud", "/cloud_raw");
        m_odomFrameId       = this->declare_parameter<std::string>("odom_frame_id", "odom");
        m_useTf             = this->declare_parameter<bool>("use_tf", true);
        m_useTfTopic        = this->declare_parameter<bool>("use_tf_topic", false);
        m_inTfTopic         = this->declare_parameter<std::string>("in_tf_topic", "/tf_stamped");
        
        m_tdfGridSizeX_low  = this->declare_parameter<double>("tdfGridSizeX_low", 10.0);
        m_tdfGridSizeX_high = this->declare_parameter<double>("tdfGridSizeX_high", 10.0);
        m_tdfGridSizeY_low  = this->declare_parameter<double>("tdfGridSizeY_low", 10.0);
        m_tdfGridSizeY_high = this->declare_parameter<double>("tdfGridSizeY_high", 10.0);
        m_tdfGridSizeZ_low  = this->declare_parameter<double>("tdfGridSizeZ_low", 10.0);
        m_tdfGridSizeZ_high = this->declare_parameter<double>("tdfGridSizeZ_high", 10.0);
        m_tdfGridRes        = this->declare_parameter<double>("tdf_grid_res", 0.10);
        m_tdfMaxCells       = this->declare_parameter<double>("tdf_max_cells", 10000.0);
        m_minRange          = this->declare_parameter<double>("min_range", 1.0);
        m_maxRange          = this->declare_parameter<double>("max_range", 100.0);
        m_PcDownsampling    = this->declare_parameter<int>("pc_downsampling", 1);
        m_occMinHits        = this->declare_parameter<int>("occ_min_hits", 1);
        m_binsAz = this->declare_parameter<int>("bins_az", 40);
        m_binsEl = this->declare_parameter<int>("bins_el", 40);
        m_shadowRadius = this->declare_parameter<int>("shadow_radius", 6);
        m_distanceMode = this->declare_parameter<std::string>("distance_mode", "L1");
        m_kernelSize   = this->declare_parameter<int>("kernel_size", 11);
        
        if (m_kernelSize % 2 == 0) {
            RCLCPP_WARN(this->get_logger(), "Kernel size must be odd! Forcing %d -> %d.", m_kernelSize, m_kernelSize + 1);
            m_kernelSize++;
        }

        RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Initializing DB-TSDF Node with Parameters:");
        RCLCPP_INFO(this->get_logger(), " "); 
        
        RCLCPP_INFO(this->get_logger(), "  Grid Params:");
        RCLCPP_INFO(this->get_logger(), "    Resolution:     %.3f m", m_tdfGridRes);
        RCLCPP_INFO(this->get_logger(), "    Max Cells:      %d", (int)m_tdfMaxCells);
        
        RCLCPP_INFO(this->get_logger(), "  Kernel Params:");
        RCLCPP_INFO(this->get_logger(), "    Kernel Size:    %dx%dx%d", m_kernelSize, m_kernelSize, m_kernelSize);
        RCLCPP_INFO(this->get_logger(), "    Occ. Min. Hits: %d", m_occMinHits);
        RCLCPP_INFO(this->get_logger(), "    Shadow Radius:  %d voxels", m_shadowRadius);
        RCLCPP_INFO(this->get_logger(), "    Distance Mode:  %s", m_distanceMode.c_str());

        RCLCPP_INFO(this->get_logger(), "  Filtering Params:");
        RCLCPP_INFO(this->get_logger(), "    Downsampling:   1 in every %d points", m_PcDownsampling);
        RCLCPP_INFO(this->get_logger(), "    Range (Min/Max):  %.1f m / %.1f m", m_minRange, m_maxRange);
        
        RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");

        // TF buffer and listener
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        
        // Publishers
        m_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 100);

        // Subscriptions
        auto qos_keepall_reliable = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();

        m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic, 
            qos_keepall_reliable,
            std::bind(&TSDFNode::pointcloudCallback, this, std::placeholders::_1));

        if (m_useTf && m_useTfTopic) {
            // Legacy Mode: Subscribe to the custom TF topic
            RCLCPP_INFO(this->get_logger(), "Using Legacy TF Mode: subscribing to '%s'", m_inTfTopic.c_str());
            m_tfSub = this->create_subscription<geometry_msgs::msg::TransformStamped>(  
                m_inTfTopic, 
                qos_keepall_reliable,
                std::bind(&TSDFNode::tfCallback, this, std::placeholders::_1));
        } else if (m_useTf) {
            // Generic Mode: Listener is already active
            RCLCPP_INFO(this->get_logger(), "Using Generic TF Mode (listening to /tf)");
        }

        save_service_pcd_ = this->create_service<std_srvs::srv::Trigger>( "/save_grid_pcd",
            std::bind(&TSDFNode::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2));

        save_service_ply_ = this->create_service<std_srvs::srv::Trigger>( "/save_grid_ply",
            std::bind(&TSDFNode::saveGridPLY, this, std::placeholders::_1, std::placeholders::_2));
        
        save_service_csv_ = this->create_service<std_srvs::srv::Trigger>( "/save_grid_csv",
            std::bind(&TSDFNode::saveSubgridCSV, this, std::placeholders::_1, std::placeholders::_2));

        save_service_mesh_ = this->create_service<std_srvs::srv::Trigger>( "/save_grid_mesh",
            std::bind(&TSDFNode::saveGridMesh, this, std::placeholders::_1, std::placeholders::_2));

        // TDF grid allocation
        m_grid3d.setup(m_tdfGridSizeX_low, m_tdfGridSizeX_high,
                    m_tdfGridSizeY_low, m_tdfGridSizeY_high,
                    m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
                    m_tdfGridRes,
                    m_kernelSize, 
                    m_occMinHits,
                    m_binsAz,         
                    m_binsEl,         
                    m_shadowRadius,   
                    m_distanceMode,
                    m_tdfMaxCells);

        std::cout << "DB-TSDF is ready to execute! " << std::endl;
        std::cout << "Grid Created. Size: " 
                << fabs(m_tdfGridSizeX_high - m_tdfGridSizeX_low) << " x " 
                << fabs(m_tdfGridSizeY_high - m_tdfGridSizeY_low) << " x " 
                << fabs(m_tdfGridSizeZ_high - m_tdfGridSizeZ_low) << "." 
                << std::endl;
    }

    ~TSDFNode(){
        RCLCPP_INFO(this->get_logger(), "Node closed successfully.");   
    }

private:

    // Parameters
    std::string m_inCloudTopic;
    std::string m_odomFrameId;
    bool m_useTf{true};
    double m_minRange, m_maxRange;
    int m_PcDownsampling;
    int m_kernelSize;
    int m_occMinHits;
    int m_binsAz;
    int m_binsEl;
    int m_shadowRadius;
    std::string m_distanceMode;
    std::string m_inTfTopic; // For legacy mode
    bool m_useTfTopic{false};  // For legacy mode

    // TDF grid and geometry
    TSDF3D16 m_grid3d;
    double m_tdfGridSizeX_low, m_tdfGridSizeX_high, 
           m_tdfGridSizeY_low, m_tdfGridSizeY_high, 
           m_tdfGridSizeZ_low, m_tdfGridSizeZ_high, 
           m_tdfGridRes, m_tdfMaxCells;

    // TF data (for Legacy Mode)
    std::deque<geometry_msgs::msg::TransformStamped> m_tfHist;
    rclcpp::Duration m_maxSkew{0, 100'000'000}; // 100ms
    std::mutex m_tfMutex;


    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_tfSub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_pcd_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_ply_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_csv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_mesh_;

    // TF management
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    // Callbacks and helpers
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
    void tfCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg);
    Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);
    void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridPLY(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveSubgridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};


void TSDFNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
    auto start = std::chrono::steady_clock::now();
    static size_t counter = 0;
    counter++;

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    rclcpp::Time t_query = cloud->header.stamp;
    if (t_query.nanoseconds() == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Cloud timestamp is 0. Using latest available transform (rclcpp::Time(0)).");
        t_query = rclcpp::Time(0);
    }

    if (m_useTf) {
        if (m_useTfTopic) {
            // --- LEGACY MODE: Manual TF deque search (for college bag) ---
            std::lock_guard<std::mutex> lock(m_tfMutex);
            if (m_tfHist.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                    "No TF received yet on LEGACY topic %s -> discarding cloud",
                                    m_inTfTopic.c_str());
                return;
            }
            
            auto best_it = m_tfHist.begin();
            auto best_dt = rclcpp::Duration::from_nanoseconds(
                std::llabs((t_query - best_it->header.stamp).nanoseconds()));

            for (auto it = std::next(m_tfHist.begin()); it != m_tfHist.end(); ++it) {
                auto dt = rclcpp::Duration::from_nanoseconds(
                    std::llabs((t_query - it->header.stamp).nanoseconds()));
                if (dt < best_dt) { 
                    best_dt = dt; 
                    best_it = it; 
                }
            }

            if (best_dt > m_maxSkew) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                    "Best legacy TF deltaT (%.3f ms) > max_skew -> discarding cloud",
                                    best_dt.seconds()*1e3);
                return;
            }
            T = getTransformMatrix(*best_it);

        } else {
            // --- GENERIC MODE: tf2_ros::Buffer (for mai_city bag) ---
            try {
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg = m_tfBuffer->lookupTransform(
                    m_odomFrameId,           
                    cloud->header.frame_id, 
                    t_query,                 
                    rclcpp::Duration(0, 100000000) 
                );
                T = getTransformMatrix(tf_msg);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Could not transform %s to %s: %s",
                    cloud->header.frame_id.c_str(), m_odomFrameId.c_str(), ex.what()
                );
                return;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_in;
    pcl::fromROSMsg(*cloud, pcl_in);

    pcl::PointCloud<pcl::PointXYZ> pcl_filtered;
    pcl_filtered.reserve(pcl_in.size());
    
    const double min_sq = m_minRange * m_minRange;
    const double max_sq = m_maxRange * m_maxRange;
    int cnt = 0;

    for (const auto &p : pcl_in) {
        const double d2 = p.x*p.x + p.y*p.y + p.z*p.z; 
        if (d2 < min_sq || d2 > max_sq) continue;
        if (cnt++ % m_PcDownsampling) continue;
        pcl_filtered.points.push_back(p);
    }
    
    pcl_filtered.width = pcl_filtered.points.size();
    pcl_filtered.height = 1;
    pcl_filtered.is_dense = true;

    pcl::PointCloud<pcl::PointXYZ> pcl_out;
    pcl::transformPointCloud(pcl_filtered, pcl_out, T); 

    std::vector<pcl::PointXYZ> pts_global(pcl_out.points.begin(), pcl_out.points.end());
    Eigen::Vector3f sensor_position = T.block<3,1>(0,3);
    m_grid3d.loadCloud(pts_global, sensor_position);

    sensor_msgs::msg::PointCloud2 cloud_corrected;
    pcl::toROSMsg(pcl_out, cloud_corrected);
    cloud_corrected.header = cloud->header;
    cloud_corrected.header.frame_id = m_odomFrameId; 
    m_cloudPub->publish(cloud_corrected);
    
    auto end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double,std::milli>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "Received frame #%zu · time = %.3f ms", counter, ms);
}


void TSDFNode::tfCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_tfMutex);
    m_tfHist.push_back(*msg);    
    while (m_tfHist.size() > 100)     
        m_tfHist.pop_front(); 
}

Eigen::Matrix4f TSDFNode::getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped){
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        Eigen::Quaternionf q(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z
        );
        Eigen::Matrix3f rotation = q.toRotationMatrix();

        Eigen::Vector3f translation(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z
        );

        transform.block<3,3>(0,0) = rotation;
        transform.block<3,1>(0,3) = translation;

        return transform;
}


// ===================== Export grid as PCD ===================== //
// ros2 service call /save_grid_pcd std_srvs/srv/Trigger
void TSDFNode::saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request to save PCD. Starting in a separate thread...");

    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generating PCD...");
        m_grid3d.exportGridToPCD("grid_data.pcd",1); 
        RCLCPP_INFO(this->get_logger(), "PCD saved successfully.");
    }).detach();

    response->success = true;
    response->message = "PCD export started in the background.";
}

// ===================== Export grid as PLY ===================== //
// ros2 service call /save_grid_ply std_srvs/srv/Trigger
void TSDFNode::saveGridPLY(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request to save PLY. Starting in a separate thread...");

    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generating PLY...");
        m_grid3d.exportGridToPLY("grid_data.ply", 1); 
        RCLCPP_INFO(this->get_logger(), "PLY saved successfully.");
    }).detach();

    response->success = true;
    response->message = "PLY export started in the background.";
}

// ===================== Export grid as CSV ===================== //
// ros2 service call /save_grid_csv std_srvs/srv/Trigger
void TSDFNode::saveSubgridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request to save CSV. Starting in a separate thread...");

    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generating CSV...");
        m_grid3d.exportSubgridToCSV("grid_data.csv", 1); 
        RCLCPP_INFO(this->get_logger(), "CSV saved successfully.");
    }).detach();

    response->success = true;
    response->message = "CSV export started in the background.";
}

// ===================== Export grid as STL ===================== //
// ros2 service call /save_grid_mesh std_srvs/srv/Trigger "{}"
void TSDFNode::saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request>, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response){

    RCLCPP_INFO(this->get_logger(),"Received request to save Mesh. Starting in background...");

    float iso = 0.00f;

    std::thread([this, iso]() {
        try {
            m_grid3d.exportMesh("mesh.stl", iso); 
            RCLCPP_INFO(this->get_logger(), "Mesh saved successfully to mesh.stl (iso=%.3f)", iso);
            } 
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),"Failed to extract mesh: %s", e.what());
        }
    }).detach();

    response->success = true;
    response->message = "Mesh export started in background.";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<TSDFNode>("db_tsdf_node");
        rclcpp::spin(node);

    } catch (const std::exception &e) {
        std::cerr << "Error creating or running the node: " << e.what() << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}
