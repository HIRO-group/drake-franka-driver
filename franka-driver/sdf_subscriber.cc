#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include "shared_memory.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace bip = boost::interprocess;
using json = nlohmann::json;

// ROS2 subscriber node that listens to cluster ESDF results and writes them to shared memory
class SDFSubscriber : public rclcpp::Node {
public:
    SDFSubscriber()
        : Node("cluster_esdf_listener")
    {
        // Create ROS2 subscription to /nvblox/esdf_results
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/nvblox/esdf_results",
            10,
            std::bind(&SDFSubscriber::listenerCallback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    // Callback called whenever a new message is received on /nvblox/esdf_results
    void listenerCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Re-open shared memory each callback so we always use the segment
        // created by panda_status_drake_subscriber_main (which removes+recreates on startup)
        bip::managed_shared_memory shm_segment;
        SharedMemoryData* shm = nullptr;
        try {
            shm_segment = bip::managed_shared_memory(bip::open_only, "MySharedMemory");
            shm = shm_segment.find<SharedMemoryData>("SharedData").first;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Shared memory not available yet: %s", e.what());
            return;
        }
        if (!shm) {
            RCLCPP_WARN(this->get_logger(), "SharedData not found in shared memory");
            return;
        }

        try {
            // Parse the JSON string from the ROS message
            json j = json::parse(msg->data);

            // Lock shared memory mutex for thread/process safety
            bip::scoped_lock<bip::interprocess_mutex> lock(shm->mutex);

            // Clear previous cluster data
            shm->clusters.clear();

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
                shm->clusters.push_back((double)lbl);
                shm->clusters.push_back(c[0]);
                shm->clusters.push_back(c[1]);
                shm->clusters.push_back(c[2]);
                shm->clusters.push_back(p0[0]);
                shm->clusters.push_back(p0[1]);
                shm->clusters.push_back(p0[2]);
                shm->clusters.push_back(p1[0]);
                shm->clusters.push_back(p1[1]);
                shm->clusters.push_back(p1[2]);
                shm->clusters.push_back(r);
            }

            // Set flag indicating that new cluster data is available
            shm->clusters_ready = true;

            // Log centroids to file
            std::ofstream cluster_log("sdf_cluster_log.txt", std::ios::app);
            if (cluster_log.is_open()) {
                size_t num_clusters = shm->clusters.size() / 11;
                cluster_log << "--- " << num_clusters << " clusters ---\n";
                for (size_t ci = 0; ci + 10 < shm->clusters.size(); ci += 11) {
                    cluster_log << "cluster " << ci / 11
                                << ": label=" << shm->clusters[ci]
                                << " centroid=(" << shm->clusters[ci+1]
                                << ", " << shm->clusters[ci+2]
                                << ", " << shm->clusters[ci+3] << ")\n";
                }
                cluster_log.close();
            }

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
