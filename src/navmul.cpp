#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("navmul_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Navigation Node...");
        
        // Initialize publishers
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker", 10);

        // Create action client for navigation
        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        // Wait for action server
        while (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for navigation action server...");
        }

        RCLCPP_INFO(this->get_logger(), "Navigation server found!");
        
        // Set initial pose
        setInitialPose(0.0, 0.0, 0.0);
        
        // Start the navigation loop
        startNavigation();
    }

private:
    // Checkpoint structure definition
    struct Checkpoint {
        double x;
        double y;
        std::string name;
    };

    // Define checkpoints
    std::vector<Checkpoint> checkpoints = {
        {1.0, 1.0, "Checkpoint 1"},
        {2.0, 2.0, "Checkpoint 2"},
        {0.0, 0.0, "Home"}
    };

    void setInitialPose(double x, double y, double theta) {
        auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = this->now();
        
        initial_pose.pose.pose.position.x = x;
        initial_pose.pose.pose.position.y = y;
        initial_pose.pose.pose.position.z = 0.0;
        
        initial_pose.pose.pose.orientation.w = cos(theta/2);
        initial_pose.pose.pose.orientation.z = sin(theta/2);
        
        for(size_t i = 0; i < 36; ++i) {
            initial_pose.pose.covariance[i] = 0.0;
        }
        initial_pose.pose.covariance[0] = 0.25;
        initial_pose.pose.covariance[7] = 0.25;
        initial_pose.pose.covariance[35] = 0.06853892326654787;

        RCLCPP_INFO(this->get_logger(), "Publishing initial pose...");
        initial_pose_pub_->publish(initial_pose);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void startNavigation() {
        while (rclcpp::ok()) {
            // Get number of goal points from user
            int num_points;
            std::cout << "\nEnter number of goal points to visit: ";
            std::cin >> num_points;
            
            if (num_points <= 0) {
                RCLCPP_WARN(this->get_logger(), "Invalid number of points. Please enter a positive number.");
                continue;
            }
            
            // Collect all points first
            std::vector<std::pair<double, double>> goals;
            for (int i = 0; i < num_points; i++) {
                double x, y;
                std::cout << "Enter target X coordinate for point " << (i+1) << ": ";
                std::cin >> x;
                std::cout << "Enter target Y coordinate for point " << (i+1) << ": ";
                std::cin >> y;
                goals.push_back({x, y});
            }
            
            // Navigate to each point sequentially
            for (size_t i = 0; i < goals.size(); i++) {
                double x = goals[i].first;
                double y = goals[i].second;
                
                current_goal_x_ = x;
                current_goal_y_ = y;

                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.header.stamp = this->now();
                goal_msg.pose.pose.position.x = x;
                goal_msg.pose.pose.position.y = y;
                goal_msg.pose.pose.orientation.w = 1.0;

                RCLCPP_INFO(this->get_logger(), "Navigating to point %zu: x=%.2f, y=%.2f", i+1, x, y);

                auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
                send_goal_options.goal_response_callback = 
                    std::bind(&NavigationNode::goalResponseCallback, this, std::placeholders::_1);
                send_goal_options.feedback_callback =
                    std::bind(&NavigationNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
                send_goal_options.result_callback =
                    std::bind(&NavigationNode::resultCallback, this, std::placeholders::_1);

                auto goal_handle_future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
                
                goal_completed_ = false;
                while (rclcpp::ok() && !goal_completed_) {
                    rclcpp::spin_some(this->get_node_base_interface());
                    std::this_thread::sleep_for(100ms);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "All %d points visited successfully!", num_points);
        }
    }

    void showMarker(double x, double y) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "goal_markers";
        marker.id = marker_count_++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.5;
        
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.lifetime = rclcpp::Duration::from_seconds(5.0);
        
        marker_pub_->publish(marker);
    }

    void publishCheckpointMarker(double x, double y, const std::string& name) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "checkpoint_markers";
        marker.id = marker_count_++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        
        // Make the checkpoint marker larger and taller
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 1.0;
        
        // Bright blue color for checkpoints
        marker.color.r = 0.0f;
        marker.color.g = 0.5f;
        marker.color.b = 1.0f;
        marker.color.a = 0.8f;
        
        marker.lifetime = rclcpp::Duration::from_seconds(5);  // Show for 5 seconds
        
        // Add text marker for checkpoint name
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = marker.header;
        text_marker.ns = "checkpoint_labels";
        text_marker.id = marker_count_++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        text_marker.pose.position = marker.pose.position;
        text_marker.pose.position.z = 1.2;  // Place text above the cylinder
        
        text_marker.text = name;
        
        text_marker.scale.z = 0.3;  // Text size
        
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0f;
        
        text_marker.lifetime = marker.lifetime;
        
        marker_pub_->publish(marker);
        marker_pub_->publish(text_marker);
    }

    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted!");
        }
    }

    void feedbackCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr&,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
    }

    void resultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
        goal_completed_ = true;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
                showMarker(current_goal_x_, current_goal_y_);
                
                // Check if this position is a checkpoint
                for (const auto& checkpoint : checkpoints) {
                    if (std::abs(current_goal_x_ - checkpoint.x) < 0.1 && 
                        std::abs(current_goal_y_ - checkpoint.y) < 0.1) {
                        RCLCPP_INFO(this->get_logger(), "Reached %s!", checkpoint.name.c_str());
                        publishCheckpointMarker(checkpoint.x, checkpoint.y, checkpoint.name);
                    }
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    bool goal_completed_ = false;
    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;
    int marker_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}