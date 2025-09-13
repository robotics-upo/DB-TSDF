#include <vector>
#include <ctime>
#include <algorithm>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2/transform_datatypes.h"

#include "pcl_ros/transforms.hpp"
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <dlo3d/tdf3d_32.hpp>

#include <memory>      
#include "mesh/vtk_mesh_extractor.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp> 
#include <mutex> 
#include <pcl/common/transforms.h> 
#include <deque> 
#include <cstdlib>

using std::isnan;
   
class DLO3DNode : public rclcpp::Node
{
public:

//!Default contructor
    DLO3DNode(const std::string &node_name)
        : Node(node_name)
    {
        // Read DLO3D parameters
        m_inCloudTopic     = this->declare_parameter<std::string>("in_cloud", "/cloud_raw");
        m_useTf            = this->declare_parameter<bool>("use_tf", true);
        m_inTfTopic        = this->declare_parameter<std::string>("in_tf", "/cloud_tf");  
        m_baseFrameId      = this->declare_parameter<std::string>("base_frame_id", "base_link");
        m_odomFrameId      = this->declare_parameter<std::string>("odom_frame_id", "odom");

        m_tdfGridSizeX_low  = this->declare_parameter<double>("tdfGridSizeX_low", 10.0);
        m_tdfGridSizeX_high = this->declare_parameter<double>("tdfGridSizeX_high", 10.0);
        m_tdfGridSizeY_low  = this->declare_parameter<double>("tdfGridSizeY_low", 10.0);
        m_tdfGridSizeY_high = this->declare_parameter<double>("tdfGridSizeY_high", 10.0);
        m_tdfGridSizeZ_low  = this->declare_parameter<double>("tdfGridSizeZ_low", 10.0);
        m_tdfGridSizeZ_high = this->declare_parameter<double>("tdfGridSizeZ_high", 10.0);
        m_tdfGridRes        = this->declare_parameter<double>("tdf_grid_res", 0.10);
        m_minRange          = this->declare_parameter<double>("min_range", 1.0);
        m_maxRange          = this->declare_parameter<double>("max_range", 100.0);
        m_PcDownsampling    = this->declare_parameter<int>("pc_downsampling", 1);
        
        //Init buffers
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        
        // Launch publishers
        m_keyframePub = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 100);
        m_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 100);
        slice_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_slice", 100);
        slice_step_ = static_cast<float>(m_tdfGridRes);
        slice_z_    = static_cast<float>(m_tdfGridSizeZ_low) + slice_step_*0.5f;
        slice_dir_  = +1;
        using namespace std::chrono_literals;
        slice_timer_ = this->create_wall_timer(300ms, std::bind(&DLO3DNode::publishSliceCB, this));

        auto qos_keepall_reliable = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();

        m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic, qos_keepall_reliable,
            std::bind(&DLO3DNode::pointcloudCallback, this, std::placeholders::_1));

        m_tfSub = this->create_subscription<geometry_msgs::msg::TransformStamped>(  
            m_inTfTopic, qos_keepall_reliable,
            std::bind(&DLO3DNode::tfCallback, this, std::placeholders::_1));

        // Create Services
        save_service_csv_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_csv",
            std::bind(&DLO3DNode::saveGridCSV, this, std::placeholders::_1, std::placeholders::_2)
        );

        save_service_pcd_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_pcd",
            std::bind(&DLO3DNode::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2)
        );

        save_service_mesh_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_mesh",
            std::bind(&DLO3DNode::saveGridMesh, this,
                    std::placeholders::_1, std::placeholders::_2)
        );

        // TDF grid Setup
        m_grid3d.setup(m_tdfGridSizeX_low, m_tdfGridSizeX_high,
                    m_tdfGridSizeY_low, m_tdfGridSizeY_high,
                    m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
                    m_tdfGridRes);

        std::cout << "DLO3D is ready to execute! " << std::endl;
        std::cout << "Grid Created. Size: " 
                << fabs(m_tdfGridSizeX_low) + fabs(m_tdfGridSizeX_high) << " x " 
                << fabs(m_tdfGridSizeY_low) + fabs(m_tdfGridSizeY_high) << " x " 
                << fabs(m_tdfGridSizeZ_low) + fabs(m_tdfGridSizeZ_high) << "." 
                << std::endl;
    }

    // Default Destructor
    ~DLO3DNode(){

        RCLCPP_INFO(this->get_logger(), "Node closed successfully.");   
    }

private:

    // ROS2 parameters
    std::string m_inCloudTopic;
    std::string m_inCloudAuxTopic;
    std::string m_baseFrameId;
    std::string m_odomFrameId;
    std::string m_inTfTopic;
    bool m_useTf{true};
    double m_minRange, m_maxRange;

    // 3D distance grid
    TDF3D32 m_grid3d;
    double m_tdfGridSizeX_low, m_tdfGridSizeX_high, m_tdfGridSizeY_low, m_tdfGridSizeY_high, m_tdfGridSizeZ_low, m_tdfGridSizeZ_high, m_tdfGridRes;

    int m_PcDownsampling;

    // Slice variables
    double slice_z0_{1.0};          
    double slice_thickness_{0.10};
    float slice_z_;            
    float slice_step_;           
    int   slice_dir_;            
    
    // Transformations
    geometry_msgs::msg::TransformStamped m_staticTfPointCloud, m_staticTfPointCloudAux, m_latestTf; 
    std::deque<geometry_msgs::msg::TransformStamped> m_tfHist;
    rclcpp::Duration m_maxSkew{0, 100'000'000}; 
    std::mutex m_tfMutex;


    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub_aux;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_keyframePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slice_pub_;
    rclcpp::TimerBase::SharedPtr slice_timer_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_tfSub;

    // ROS2 transform management
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_csv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_pcd_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_mesh_;

    // Function declarations
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);

    void tfCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg);
    
    Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);
    
    void saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void publishSliceCB();

};

void DLO3DNode::saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Received request to save CSV. Starting in a separate thread...");
    
    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generating CSV...");
        m_grid3d.exportGridToCSV("grid_data.csv",m_tdfGridSizeX_low, m_tdfGridSizeX_high,
                    m_tdfGridSizeY_low, m_tdfGridSizeY_high,
                    m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
                      1);
        RCLCPP_INFO(this->get_logger(), "CSV saved successfully..");
    }).detach();

    response->success = true;
    response->message = "CSV export started in the background.";

}

// ros2 service call /save_grid_pcd std_srvs/srv/Trigger


void DLO3DNode::saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request to save PCD. Starting in a separate thread...");

    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generating PCD...");
        m_grid3d.exportGridToPCD("grid_data.pcd",1); 
        RCLCPP_INFO(this->get_logger(), "PCD saved successfully.");
    }).detach();

    response->success = true;
    response->message = "Exportación del PCD iniciada en segundo plano.";
}

void DLO3DNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
    {
        auto start = std::chrono::steady_clock::now();
        static size_t counter = 0;
        counter++;

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        if (m_useTf)
        {
            std::lock_guard<std::mutex> lock(m_tfMutex);

            if (m_tfHist.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                    "Sin TF aún en %s → nube descartada",
                                    m_inTfTopic.c_str());
                return;
            }

            // Usa SIEMPRE la última TF recibida
            // const auto& tf_last = m_tfHist.back();
            // T = getTransformMatrix(tf_last);

            rclcpp::Time t_cloud = cloud->header.stamp;
            if (t_cloud.nanoseconds() == 0) t_cloud = this->get_clock()->now();

            // Selecciona la TF con |Δt| mínimo
            auto best_it = m_tfHist.begin();
            auto best_dt = rclcpp::Duration::from_nanoseconds(
                std::llabs((t_cloud - best_it->header.stamp).nanoseconds()));

            for (auto it = std::next(m_tfHist.begin()); it != m_tfHist.end(); ++it) {
                auto dt = rclcpp::Duration::from_nanoseconds(
                    std::llabs((t_cloud - it->header.stamp).nanoseconds()));
                if (dt < best_dt) { best_dt = dt; best_it = it; }
            }

            // Descarta si el desfase supera el umbral permitido
            if (best_dt > m_maxSkew) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                    "Δt TF–cloud = %.3f ms > límite, nube descartada",
                                    best_dt.seconds()*1e3);
                return;
            }

            T = getTransformMatrix(*best_it);

        }


        const float base_x = T(0,3);
        const float base_y = T(1,3);
        const float base_z = T(2,3);
        
        pcl::PointCloud<pcl::PointXYZ> pcl_in, pcl_out;
        pcl::fromROSMsg(*cloud, pcl_in);
        pcl::transformPointCloud(pcl_in, pcl_out, T); 

        std::vector<pcl::PointXYZ> pts;
        pts.reserve(pcl_out.size());
        const double min_sq = m_minRange * m_minRange;
        const double max_sq = m_maxRange * m_maxRange;
        int cnt = 0;
        
        pcl::PointCloud<pcl::PointXYZ> pcl_filtered;
        pcl_filtered.reserve(pcl_out.size());

        for (const auto &p : pcl_out) {
            const double dx = p.x - base_x;
            const double dy = p.y - base_y;
            const double dz = p.z - base_z;
            const double d2 = dx*dx + dy*dy + dz*dz; 
            if (d2 < min_sq || d2 > max_sq) continue;
            if (cnt++ % m_PcDownsampling)   continue;
            pts.push_back(p);
            pcl_filtered.push_back(p);
        }

        m_grid3d.loadCloud(pts);

        sensor_msgs::msg::PointCloud2 cloud_corrected;
        // pcl::toROSMsg(pcl_out, cloud_corrected);
        pcl::toROSMsg(pcl_filtered, cloud_corrected);
        cloud_corrected.header   = cloud->header;
        cloud_corrected.header.frame_id = m_odomFrameId;    
        m_cloudPub->publish(cloud_corrected);
        

        auto end = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double,std::milli>(end - start).count();
        RCLCPP_INFO(this->get_logger(), "Received frame #%zu · time = %.3f", counter, ms);
    }


// //! 3D point-cloud callback
// void DLO3DNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
// {
//     auto start = std::chrono::steady_clock::now();
//     static size_t counter = 0;
//     counter++;

//     // 1) Centro de la esfera (opcionalmente desde /tf_ts)
//     float base_x = 0.f, base_y = 0.f, base_z = 0.f;
//     if (m_useTf) {
//         std::lock_guard<std::mutex> lock(m_tfMutex);
//         if (!m_tfHist.empty()) {
//             rclcpp::Time t_cloud = cloud->header.stamp;
//             auto best_it = m_tfHist.begin();
//             auto best_dt = rclcpp::Duration::from_nanoseconds(
//                 std::llabs((t_cloud - best_it->header.stamp).nanoseconds()));
//             for (auto it = m_tfHist.begin(); it != m_tfHist.end(); ++it) {
//                 auto dt = rclcpp::Duration::from_nanoseconds(
//                     std::llabs((t_cloud - it->header.stamp).nanoseconds()));
//                 if (dt < best_dt) { best_dt = dt; best_it = it; }
//             }
//             // SIEMPRE tomar ese TF para el centro de la esfera
//             base_x = best_it->transform.translation.x;
//             base_y = best_it->transform.translation.y;
//             base_z = best_it->transform.translation.z;
//         }
//     }

//     // 2) Nube: NO transformar
//     pcl::PointCloud<pcl::PointXYZ> pcl_in, pcl_out;
//     pcl::fromROSMsg(*cloud, pcl_in);
//     pcl_out = pcl_in;

//     const double min_sq = m_minRange * m_minRange;
//     const double max_sq = m_maxRange * m_maxRange;

//     pcl::PointCloud<pcl::PointXYZ> pcl_filtered;
//     pcl_filtered.reserve(pcl_out.size());

//     std::vector<pcl::PointXYZ> pts;
//     pts.reserve(pcl_out.size());

//     int cnt = 0;
//     for (const auto &p : pcl_out) {
//         const double dx = p.x - base_x;
//         const double dy = p.y - base_y;
//         const double dz = p.z - base_z;
//         const double d2 = dx*dx + dy*dy + dz*dz;
//         if (d2 < min_sq || d2 > max_sq) continue;
//         if (cnt++ % m_PcDownsampling)   continue;
//         pts.push_back(p);
//         pcl_filtered.push_back(p);
//     }

//     m_grid3d.loadCloud(pts);

//     sensor_msgs::msg::PointCloud2 cloud_corrected;
//     pcl::toROSMsg(pcl_filtered, cloud_corrected);
//     cloud_corrected.header = cloud->header;          
//     cloud_corrected.header.frame_id = "odom";        
//     m_cloudPub->publish(cloud_corrected);

//     auto end = std::chrono::steady_clock::now();
//     double ms = std::chrono::duration<double,std::milli>(end - start).count();
//     RCLCPP_INFO(this->get_logger(), "Received frame #%zu · time = %.3f", counter, ms);
// }

void DLO3DNode::tfCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_tfMutex);
    m_tfHist.push_back(*msg);    
    while (m_tfHist.size() > 100)     
        m_tfHist.pop_front(); 
}

//! Auxiliar Functions
Eigen::Matrix4f DLO3DNode::getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped){
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

// ros2 service call /save_grid_mesh std_srvs/srv/Trigger "{ }"

void DLO3DNode::saveGridMesh(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
    RCLCPP_INFO(this->get_logger(),
                "Received request to save Mesh. Starting in background...");

    float iso = 0.00f;

    std::thread([this, iso]() {
        try {
            VTKMeshExtractor::extract(m_grid3d, "mesh.stl", iso);
            RCLCPP_INFO(this->get_logger(), "Mesh saved successfully to mesh.stl (iso=%.3f)", iso);
            } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),
                        "Failed to extract mesh: %s", e.what());
        }
    }).detach();

    response->success = true;
    response->message = "Mesh export started in background.";
}



// void DLO3DNode::saveGridMesh(
//     const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
//     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
// {
//   RCLCPP_INFO(this->get_logger(),"Saving mesh (VTK FlyingEdges, TSDF)...");
//   const float iso = 0.0f;
//   std::thread([this, iso]() {
//     try {
//       VTKMeshExtractor::extract(m_grid3d, "/home/ros/ros2_ws/mesh.ply", iso);
//       RCLCPP_INFO(this->get_logger(), "Mesh saved to /home/ros/ros2_ws/mesh.ply");
//     } catch (const std::exception &e) {
//       RCLCPP_ERROR(this->get_logger(), "Mesh export failed: %s", e.what());
//     }
//   }).detach();
//   response->success = true;
//   response->message = "PLY export started.";
// }

void DLO3DNode::publishSliceCB()
{
    nav_msgs::msg::OccupancyGrid slice;
    m_grid3d.buildGridSliceMsg(slice_z_, slice);            

    slice_pub_->publish(slice);                  

    slice_z_ += slice_dir_ * slice_step_;

    if (slice_z_ > float(m_tdfGridSizeZ_high)) {  
        slice_z_  = float(m_tdfGridSizeZ_high);
        slice_dir_ = -1;
    } else if (slice_z_ < float(m_tdfGridSizeZ_low)) { 
        slice_z_  = float(m_tdfGridSizeZ_low);
        slice_dir_ = +1;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<DLO3DNode>("dll3d_node");
        rclcpp::spin(node);

    } catch (const std::exception &e) {
        std::cerr << "Error creating or running the node: " << e.what() << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}
