#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <vector>
#include <cstring>

using namespace std::chrono_literals;

class LidarUdpReceiver : public rclcpp::Node {
public:
    LidarUdpReceiver() : Node("lidar_udp_receiver") {
        // Declare and get parameters
        declare_parameter("port", 8089);
        declare_parameter("frame_id", "lidar_link");
        declare_parameter("scan_topic", "scan");
        declare_parameter("angle_min", -M_PI);
        declare_parameter("angle_max", M_PI);
        declare_parameter("angle_increment", 0.0174533); // approx 1 degree in radians
        declare_parameter("range_min", 0.15);            // 15cm min range
        declare_parameter("range_max", 12.0);            // 12m max range
        declare_parameter("scan_time", 0.1);             // 10Hz scan rate
        declare_parameter("publish_rate", 10.0);         // 10Hz publish rate

        port_ = get_parameter("port").as_int();
        frame_id_ = get_parameter("frame_id").as_string();
        scan_topic_ = get_parameter("scan_topic").as_string();
        angle_min_ = get_parameter("angle_min").as_double();
        angle_max_ = get_parameter("angle_max").as_double();
        angle_increment_ = get_parameter("angle_increment").as_double();
        range_min_ = get_parameter("range_min").as_double();
        range_max_ = get_parameter("range_max").as_double();
        scan_time_ = get_parameter("scan_time").as_double();
        publish_rate_ = get_parameter("publish_rate").as_double();

        // Create publisher
        scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);

        // Setup UDP socket
        if (!setupUdpSocket()) {
            RCLCPP_ERROR(get_logger(), "Failed to setup UDP socket. Exiting.");
            rclcpp::shutdown();
            return;
        }

        // Setup timer for receiving data
        double period_seconds = 1.0 / publish_rate_;
        receive_timer_ = create_wall_timer(
            std::chrono::duration<double>(period_seconds),
            std::bind(&LidarUdpReceiver::receiveAndPublish, this));

        RCLCPP_INFO(get_logger(), "Lidar UDP receiver initialized. Listening on port %d", port_);
    }

    ~LidarUdpReceiver() {
        // Close socket on shutdown
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

private:
    bool setupUdpSocket() {
        // Create socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to create socket");
            return false;
        }

        // Configure socket to be non-blocking
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        // Setup address struct
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(port_);

        // Bind socket
        if (bind(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to bind socket to port %d", port_);
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        return true;
    }

    void receiveAndPublish() {
        const size_t max_buffer_size = 8192 * 16 + 12; // Max points * bytes per point + header
        std::vector<char> buffer(max_buffer_size);

        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        // Try to receive data
        ssize_t received_bytes = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                         (struct sockaddr*)&client_addr, &client_addr_len);

        if (received_bytes <= 0) {
            // No data or error
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(get_logger(), "Error receiving UDP data: %s", strerror(errno));
            }
            return;
        }

        // Log source of data
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        RCLCPP_DEBUG(get_logger(), "Received %zd bytes from %s:%d", 
                   received_bytes, client_ip, ntohs(client_addr.sin_port));

        // Process received data if we have at least header (timestamp + count)
        if (received_bytes >= 12) {
            processLidarData(buffer.data(), received_bytes);
        }
    }

    void processLidarData(const char* data, size_t size) {
        // Parse header: timestamp (8 bytes) + count (4 bytes)
        uint64_t timestamp;
        uint32_t count;

        memcpy(&timestamp, data, 8);
        memcpy(&count, data + 8, 4);

        // Calculate expected size
        size_t expected_size = 12 + (count * 16); // header + data points
        if (size < expected_size) {
            RCLCPP_WARN(get_logger(), "Received incomplete packet: %zu bytes, expected %zu bytes", 
                      size, expected_size);
            return;
        }

        // Create LaserScan message
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        
        // Set header
        scan_msg->header.stamp = this->now();
        scan_msg->header.frame_id = frame_id_;
        
        // Set scan parameters
        scan_msg->angle_min = angle_min_;
        scan_msg->angle_max = angle_max_;
        scan_msg->angle_increment = angle_increment_;
        scan_msg->time_increment = scan_time_ / static_cast<double>(count);
        scan_msg->scan_time = scan_time_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;
        
        // Resize range and intensity arrays
        size_t num_readings = static_cast<size_t>((angle_max_ - angle_min_) / angle_increment_) + 1;
        scan_msg->ranges.resize(num_readings, std::numeric_limits<float>::infinity());
        scan_msg->intensities.resize(num_readings, 0.0f);
        
        // Process each scan point
        const char* ptr = data + 12; // Skip header
        for (uint32_t i = 0; i < count; i++) {
            float angle, distance;
            uint32_t quality, flag;
            
            memcpy(&angle, ptr, 4);
            memcpy(&distance, ptr + 4, 4);
            memcpy(&quality, ptr + 8, 4);
            memcpy(&flag, ptr + 12, 4);
            
            ptr += 16;
            
            // Convert angle to radians if needed
            float angle_rad = angle * (M_PI / 180.0f);
            
            // Normalize angle to the range [-π, π]
            while (angle_rad > M_PI) angle_rad -= 2.0f * M_PI;
            while (angle_rad < -M_PI) angle_rad += 2.0f * M_PI;
            
            // Calculate index in the arrays
            int index = static_cast<int>((angle_rad - angle_min_) / angle_increment_);
            
            // Check if index is within bounds
            if (index >= 0 && index < static_cast<int>(num_readings)) {
                // Convert distance from mm to meters
                float range = distance / 1000.0f;
                
                // Only update if this range is better (closer but still valid)
                if (range >= range_min_ && range <= range_max_ && 
                    (scan_msg->ranges[index] > range || scan_msg->ranges[index] == std::numeric_limits<float>::infinity())) {
                    scan_msg->ranges[index] = range;
                    scan_msg->intensities[index] = static_cast<float>(quality);
                }
            }
        }
        
        // Publish the scan message
        scan_publisher_->publish(std::move(scan_msg));
    }

    // ROS2 members
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::TimerBase::SharedPtr receive_timer_;

    // Parameters
    int port_;
    std::string frame_id_;
    std::string scan_topic_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;
    double scan_time_;
    double publish_rate_;

    // UDP socket
    int socket_fd_ = -1;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarUdpReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}