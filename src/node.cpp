// pedestrian_listener.cpp
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "derived_object_msgs/msg/object_array.hpp"

#include <dummy_perception_publisher/msg/object.hpp>

#include <random>
#include <unordered_map>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::Shape;
using dummy_perception_publisher::msg::Object;
using std::placeholders::_1;
using namespace std::chrono_literals;

// Define the PedestrianListenerNode class which inherits from rclcpp::Node
class PedestrianListenerNode : public rclcpp::Node
{
public:
    PedestrianListenerNode() : Node("pedestrian_listener_node")
    {
        // Subscriber to the topic "pedestrian_simulator/pedestrians"
        subscription_ = this->create_subscription<derived_object_msgs::msg::ObjectArray>(
            "pedestrian_simulator/pedestrians",
            10, // Queue size
            std::bind(&PedestrianListenerNode::callback, this, _1));

        dummy_object_info_pub_ = this->create_publisher<Object>("/simulation/dummy_perception_publisher/object_info", 1);

        // Pedestrian simulator
        _ped_reset_pub = this->create_publisher<std_msgs::msg::Empty>(
            "/pedestrian_simulator/reset", 1);
        _ped_reset_to_start_pub = this->create_publisher<std_msgs::msg::Empty>(
            "/pedestrian_simulator/reset_to_start", 1);
        _ped_horizon_pub =
            this->create_publisher<std_msgs::msg::Int32>("/pedestrian_simulator/horizon", 1);
        _ped_integrator_step_pub = this->create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/integrator_step", 1);
        _ped_clock_frequency_pub = this->create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/clock_frequency", 1);
        _ped_start_client = this->create_client<std_srvs::srv::Empty>("/pedestrian_simulator/start");

        _goal_reached_sub = this->create_subscription<std_msgs::msg::Empty>(
            "/route_converter/goal_reached",
            10, // Queue size
            std::bind(&PedestrianListenerNode::goalReachedCallback, this, _1));

        _trigger_goal_pub = this->create_publisher<std_msgs::msg::Empty>(
            "/planning/mpc_planner_plugin/trigger_goal", 1);
        _autoware_position_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1);

        startSimulator();
    }

private:
    std::unordered_map<int, Object> known_pedestrians;
    bool first = true;

    // Callback function that processes incoming data from the topic
    void callback(const derived_object_msgs::msg::ObjectArray::SharedPtr msg)
    {
        if (first)
        {
            first = false;
            Object delete_all_msg;
            delete_all_msg.header.frame_id = "map";
            delete_all_msg.header.stamp = this->get_clock()->now();
            delete_all_msg.action = dummy_perception_publisher::msg::Object::DELETEALL;
            dummy_object_info_pub_->publish(delete_all_msg);
            return;
        }

        for (const auto &pedestrian : msg->objects)
        {
            if (known_pedestrians.find(pedestrian.id) == known_pedestrians.end())
            {
                // Spawn new pedestrian
                RCLCPP_INFO(this->get_logger(), "New Pedestrian, ID: %d", pedestrian.id);

                Object object{};

                // header
                object.header.frame_id = "map";
                object.header.stamp = this->get_clock()->now();

                // semantic
                object.classification.label = ObjectClassification::PEDESTRIAN;
                object.classification.probability = 1.0;

                // shape
                object.shape.type = Shape::CYLINDER;
                const double width = 0.6;
                const double length = 0.6;
                object.shape.dimensions.x = length;
                object.shape.dimensions.y = width;
                object.shape.dimensions.z = 2.0;

                // initial state
                object.initial_state.pose_covariance.pose.position.z = 0.;
                object.initial_state.pose_covariance.covariance[0] = 0.;
                object.initial_state.pose_covariance.covariance[7] = 0.;
                object.initial_state.pose_covariance.covariance[14] = 0.;
                object.initial_state.pose_covariance.covariance[35] = 0.;

                std::mt19937 gen(std::random_device{}());
                std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
                std::generate(object.id.uuid.begin(), object.id.uuid.end(), bit_eng);

                object.initial_state.pose_covariance.pose.position.x = pedestrian.pose.position.x;
                object.initial_state.pose_covariance.pose.position.y = pedestrian.pose.position.y;
                object.initial_state.pose_covariance.pose.position.z = 0.;
                object.initial_state.pose_covariance.pose.orientation = pedestrian.pose.orientation;
                object.initial_state.twist_covariance.twist.linear.x = pedestrian.twist.linear.x;
                object.initial_state.twist_covariance.twist.linear.y = 0.0;
                object.initial_state.twist_covariance.twist.linear.z = 0.0;
                object.initial_state.accel_covariance.accel.linear.x = pedestrian.accel.linear.x;
                object.initial_state.accel_covariance.accel.linear.y = 0.0;
                object.initial_state.accel_covariance.accel.linear.z = 0.0;
                object.max_velocity = pedestrian.twist.linear.x;
                object.min_velocity = pedestrian.twist.linear.x;
                object.action = Object::ADD;

                dummy_object_info_pub_->publish(object);

                known_pedestrians[pedestrian.id] = object;
            }
            else
            {
                // Move pedestrian
                auto &object = known_pedestrians[pedestrian.id];

                // header
                // object.header.frame_id = "map";
                object.header.stamp = this->get_clock()->now();

                object.initial_state.pose_covariance.pose.position.x = pedestrian.pose.position.x;
                object.initial_state.pose_covariance.pose.position.y = pedestrian.pose.position.y;
                object.initial_state.pose_covariance.pose.position.z = 0.;
                object.initial_state.pose_covariance.pose.orientation = pedestrian.pose.orientation;
                object.initial_state.twist_covariance.twist.linear.x = pedestrian.twist.linear.x;
                object.initial_state.twist_covariance.twist.linear.y = 0.0;
                object.initial_state.twist_covariance.twist.linear.z = 0.0;
                object.initial_state.accel_covariance.accel.linear.x = pedestrian.accel.linear.x;
                object.initial_state.accel_covariance.accel.linear.y = 0.0;
                object.initial_state.accel_covariance.accel.linear.z = 0.0;
                object.max_velocity = pedestrian.twist.linear.x;
                object.min_velocity = pedestrian.twist.linear.x;
                object.action = Object::MODIFY;

                dummy_object_info_pub_->publish(object);
            }
        }
    }

    void goalReachedCallback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        startExperiment();
    }

    void startSimulator()
    {
        RCLCPP_INFO(this->get_logger(), "Starting pedestrian simulator");
        while (!_ped_start_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return;
            }

            rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1. * 1e9)));
        }

        std_msgs::msg::Int32 horizon_msg;
        horizon_msg.data = 20;
        _ped_horizon_pub->publish(horizon_msg);

        std_msgs::msg::Float32 integrator_step_msg;
        integrator_step_msg.data = 0.2;
        _ped_integrator_step_pub->publish(integrator_step_msg);

        std_msgs::msg::Float32 clock_frequency_msg;
        clock_frequency_msg.data = 50.;
        _ped_clock_frequency_pub->publish(clock_frequency_msg);

        auto empty_srv = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = _ped_start_client->async_send_request(empty_srv);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {

            // _experiment_timer = std::make_unique<RosTools::Timer>(CONFIG["recording"]["timeout"].as<double>());

            // Set the vehicle pose if in simulation mode
            std_msgs::msg::Empty empty_msg;
            _ped_reset_to_start_pub->publish(empty_msg);

            startExperiment();

            return;
        }
    }

    void startExperiment()
    {
        RCLCPP_INFO(this->get_logger(), "Starting a new experiment");

        // _starting = true;
        // Publish the autoware start pose
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();
        msg.pose.pose.position.x = 94092.4765625;
        msg.pose.pose.position.y = 61838.18359375;
        msg.pose.pose.orientation.z = 0.17520679192214517;
        msg.pose.pose.orientation.w = 0.9845316551865411;

        _autoware_position_pub->publish(msg);

        // Give some time for the spawn
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 * 1e9)));

        _autoware_position_pub->publish(msg);

        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 * 1e9)));

        std_msgs::msg::Empty empty_msg;
        _trigger_goal_pub->publish(empty_msg);

        // _experiment_timer->start();

        // _starting = false;
    }

    // Subscriber object to hold the subscription
    rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _goal_reached_sub;
    rclcpp::Publisher<Object>::SharedPtr dummy_object_info_pub_;

    // Pedestrian Simulator
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _ped_reset_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _ped_reset_to_start_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _ped_horizon_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_integrator_step_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_clock_frequency_pub;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _ped_start_client;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _autoware_position_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _trigger_goal_pub;
};

// Main function to start the node
int main(int argc, char *argv[])
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the PedestrianListenerNode
    auto node = std::make_shared<PedestrianListenerNode>();

    // Keep the node spinning so it can continue processing callbacks
    rclcpp::spin(node);

    // Shutdown the ROS 2 system after the node is done
    rclcpp::shutdown();
    return 0;
}
