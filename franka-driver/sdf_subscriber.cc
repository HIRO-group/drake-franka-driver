#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include "shared_memory.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace bip = boost::interprocess;
using json = nlohmann::json;

// ROS2 subscriber node that listens to cluster ESDF results and writes them to shared memory
class SDFSubscriber : public rclcpp::Node {
public:
    SDFSubscriber()
        : Node("cluster_esdf_listener"),
          shm_segment_(bip::open_or_create, "MySharedMemory", 65536), // Create or open shared memory segment
          shm_(nullptr)
    {
        // Attempt to find existing shared memory object named "SharedData"
        auto res = shm_segment_.find<SharedMemoryData>("SharedData");
        shm_ = res.first;

        if (!shm_) {
            // If not found, construct a new SharedMemoryData object in shared memory
            auto* sm = shm_segment_.get_segment_manager();
            SharedMemoryData::ShmemAllocator alloc(sm);
            shm_ = shm_segment_.construct<SharedMemoryData>("SharedData")(alloc);
            RCLCPP_INFO(this->get_logger(), "Created new SharedMemoryData");
        } else {
            RCLCPP_INFO(this->get_logger(), "Found existing SharedMemoryData");
        }

        // Create ROS2 subscription to /nvblox/esdf_results
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/nvblox/esdf_results",
            10,
            std::bind(&SDFSubscriber::listenerCallback, this, std::placeholders::_1)
        );
    }

private:
    bip::managed_shared_memory shm_segment_;      // Shared memory segment
    SharedMemoryData* shm_;                       // Pointer to shared memory object
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    // Callback called whenever a new message is received on /nvblox/esdf_results
    void listenerCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (!shm_) return;

        try {
            // Parse the JSON string from the ROS message
            json j = json::parse(msg->data);

            // Lock shared memory mutex for thread/process safety
            bip::scoped_lock<bip::interprocess_mutex> lock(shm_->mutex);

            // Clear previous cluster data
            shm_->clusters.clear();

            // Iterate over all clusters in the JSON
            for (auto& [label, cluster] : j.items()) {
                int lbl = std::stoi(label);  // Cluster label as integer

                // Extract centroid of cluster (3 doubles: x, y, z)
                auto c = cluster["centroid"];
                // Extract capsule info: p0, p1 are points (3 doubles each), radius is double
                auto cap = cluster["capsule"];
                auto p0 = cap["p0"];
                auto p1 = cap["p1"];
                double r = cap["radius"];

                // Logging: print cluster info to console for debugging
                RCLCPP_INFO(this->get_logger(),
                    "Cluster %s centroid: (%.3f, %.3f, %.3f)",
                    label.c_str(), (double)c[0], (double)c[1], (double)c[2]);

                RCLCPP_INFO(this->get_logger(),
                    "Capsule p0:(%.2f,%.2f,%.2f) p1:(%.2f,%.2f,%.2f) r=%.3f",
                    (double)p0[0], (double)p0[1], (double)p0[2],
                    (double)p1[0], (double)p1[1], (double)p1[2],
                    r);

                // -------------------------------
                // Write data into shared memory
                // -------------------------------
                // Data layout per cluster (flattened):
                // [label, cx, cy, cz, p0x, p0y, p0z, p1x, p1y, p1z, radius]
                // Each element is a double. Multiple clusters are appended sequentially.
                shm_->clusters.push_back((double)lbl);
                shm_->clusters.push_back(c[0]);
                shm_->clusters.push_back(c[1]);
                shm_->clusters.push_back(c[2]);
                shm_->clusters.push_back(p0[0]);
                shm_->clusters.push_back(p0[1]);
                shm_->clusters.push_back(p0[2]);
                shm_->clusters.push_back(p1[0]);
                shm_->clusters.push_back(p1[1]);
                shm_->clusters.push_back(p1[2]);
                shm_->clusters.push_back(r);
            }

            // Set flag indicating that new cluster data is available
            shm_->clusters_ready = true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON/SHM error: %s", e.what());
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDFSubscriber>());
    rclcpp::shutdown();
    return 0;
}
