
#include <rclcpp/rclcpp.hpp>
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "hapthexa_msgs/msg/leg_args.hpp"

#include <thread>
#include <chrono>
using namespace std::chrono_literals;

enum leg_num
{
    front_left = 0,
    middle_left,
    rear_left,
    rear_right,
    middle_right,
    front_right,
};

class HaptHexaServo : public rclcpp::Node
{
public:
    HaptHexaServo() : Node("hapthexa_servo")
    {
        bool result = false;
        uint16_t model_number = 0;
        const char *log;

        // U2D2との通信を確立
        RCLCPP_INFO(this->get_logger(), "Initializing U2D2 and Dynamixels...");
        do {
            if (dxl_wb.init("/dev/ttyUSB0", 4*1000*1000, &log) == true) break;
            RCLCPP_WARN(this->get_logger(), log);
            RCLCPP_WARN(this->get_logger(), "Failed to initialize U2D2 and Dynamixels. retrying...");
            std::this_thread::sleep_for(5s);
        } while (true);
        RCLCPP_INFO(this->get_logger(), "Succeeded to initialize U2D2 and Dynamixels");

        // pingによる全サーボの疎通確認
        RCLCPP_INFO(this->get_logger(), "Checking connection to Dynamixels...");
        do {
            int ping_succeed_count = 0;
            char is_ping_succeed[19] = {};
            for (int id = 1; id <= 18; id++) {
                if (dxl_wb.ping(id, &model_number, &log)) {
                    ++ping_succeed_count;
                    is_ping_succeed[id-1] = 'o';
                } else {
                    is_ping_succeed[id-1] = 'x';
                }
            }
            is_ping_succeed[18] = '\0';
            RCLCPP_INFO(this->get_logger(), "\nping result: [%s]", is_ping_succeed);

            if (ping_succeed_count == 18) break;

            RCLCPP_WARN(this->get_logger(), "Failed to ping. retrying...");
            std::this_thread::sleep_for(5s);
        } while (true);
        RCLCPP_INFO(this->get_logger(), "Succeeded to ping");

        // ジョイントモードの変更
        RCLCPP_INFO(this->get_logger(), "Changing joint mode...");
        do {
            int change_succeed_count = 0;
            char is_change_succeed[19] = {};
            for (int id = 1; id <= 18; id++) {
                if (dxl_wb.jointMode(id, 0, 0, &log)) {
                    ++change_succeed_count;
                    is_change_succeed[id-1] = 'o';
                } else {
                    is_change_succeed[id-1] = 'x';
                }
            }
            is_change_succeed[18] = '\0';
            RCLCPP_INFO(this->get_logger(), "\nchange joint mode result: [%s]", is_change_succeed);

            if (change_succeed_count == 18) break;

            RCLCPP_WARN(this->get_logger(), "Failed to change joint mode. retrying...");
            std::this_thread::sleep_for(5s);
        } while (true);
        RCLCPP_INFO(this->get_logger(), "Succeeded to change joint mode");

        result = dxl_wb.initBulkWrite(&log);
        RCLCPP_INFO(this->get_logger(), "%s", log);

        result = dxl_wb.initBulkRead(&log);
        RCLCPP_INFO(this->get_logger(), "%s", log);
        // result = dxl_wb.addBulkReadParam(1, "Present_Position", &log);
        // dxl_wb.addBulkReadParam(dxl_id[0], 0, 0, &log);

        sub_[front_left] = this->create_subscription<hapthexa_msgs::msg::LegArgs>(
            "hapthexa/leg/front_left/leg_args", rclcpp::QoS(10),
            [&](hapthexa_msgs::msg::LegArgs::SharedPtr msg) {
                const char *log;
                dxl_wb.addBulkWriteParam(16, "Goal_Position", -msg->coxa_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(17, "Goal_Position", msg->femur_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(18, "Goal_Position", -msg->tibia_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.bulkWrite(&log);
            });
        sub_[middle_left] = this->create_subscription<hapthexa_msgs::msg::LegArgs>(
            "hapthexa/leg/middle_left/leg_args", rclcpp::QoS(10),
            [&](hapthexa_msgs::msg::LegArgs::SharedPtr msg) {
                const char *log;
                dxl_wb.addBulkWriteParam(13, "Goal_Position", -msg->coxa_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(14, "Goal_Position", msg->femur_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(15, "Goal_Position", -msg->tibia_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.bulkWrite(&log);
            });
        sub_[rear_left] = this->create_subscription<hapthexa_msgs::msg::LegArgs>(
            "hapthexa/leg/rear_left/leg_args", rclcpp::QoS(10),
            [&](hapthexa_msgs::msg::LegArgs::SharedPtr msg) {
                const char *log;
                dxl_wb.addBulkWriteParam(10, "Goal_Position", -msg->coxa_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(11, "Goal_Position", msg->femur_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(12, "Goal_Position", -msg->tibia_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.bulkWrite(&log);
            });
        sub_[rear_right] = this->create_subscription<hapthexa_msgs::msg::LegArgs>(
            "hapthexa/leg/rear_right/leg_args", rclcpp::QoS(10),
            [&](hapthexa_msgs::msg::LegArgs::SharedPtr msg) {
                const char *log;
                dxl_wb.addBulkWriteParam(7, "Goal_Position", -msg->coxa_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(8, "Goal_Position", -msg->femur_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(9, "Goal_Position", msg->tibia_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.bulkWrite(&log);
            });
        sub_[middle_right] = this->create_subscription<hapthexa_msgs::msg::LegArgs>(
            "hapthexa/leg/middle_right/leg_args", rclcpp::QoS(10),
            [&](hapthexa_msgs::msg::LegArgs::SharedPtr msg) {
                const char *log;
                dxl_wb.addBulkWriteParam(4, "Goal_Position", -msg->coxa_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(5, "Goal_Position", -msg->femur_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(6, "Goal_Position", msg->tibia_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.bulkWrite(&log);
            });
        sub_[front_right] = this->create_subscription<hapthexa_msgs::msg::LegArgs>(
            "hapthexa/leg/front_right/leg_args", rclcpp::QoS(10),
            [&](hapthexa_msgs::msg::LegArgs::SharedPtr msg) {
                const char *log;
                dxl_wb.addBulkWriteParam(1, "Goal_Position", -msg->coxa_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(2, "Goal_Position", -msg->femur_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.addBulkWriteParam(3, "Goal_Position", msg->tibia_arg * (2048 / M_PI) + 2048, &log);
                dxl_wb.bulkWrite(&log);
            });

        for (int i = 0; i < 6; i++)
        {
            present_leg_args_pubs[i] = this->create_publisher<hapthexa_msgs::msg::LegArgs>("hapthexa/leg/" + leg_names[i] + "/present_leg_args", rclcpp::QoS(10));
        }
        timer_ = this->create_wall_timer(10ms, [&]() {
            const char *log;
            for (uint8_t id = 1; id <= 18; id++)
            {
                dxl_wb.addBulkReadParam(id, "Present_Position", &log);
            }
            dxl_wb.bulkRead(&log);
            int32_t position[18];
            dxl_wb.getBulkReadData(position, &log);

            for (int i = 0; i < 6; i++)
            {
                auto leg_args = hapthexa_msgs::msg::LegArgs();
                for (int j = 0; j < 3; j++)
                {
                    int legnum = 5 - i;
                    int angle = position[legnum*3+j] - 2048;
                    if (j == 0) angle *= -1;
                    if (legnum <= 2)
                    {
                        if (j == 1)
                            angle *= -1;
                    }
                    else
                    {
                        if (j == 2)
                            angle *= -1;
                    }

                    double arg = angle * (M_PI/2048.0);
                    switch (j)
                    {
                    case 0:
                        leg_args.coxa_arg = arg;
                        break;
                    case 1:
                        leg_args.femur_arg = arg;
                        break;
                    case 2:
                        leg_args.tibia_arg = arg;
                        break;
                    }
                }
                present_leg_args_pubs[i]->publish(leg_args);
            }
        });
    }

private:
    DynamixelWorkbench dxl_wb;
    std::array<rclcpp::Subscription<hapthexa_msgs::msg::LegArgs>::SharedPtr, 6> sub_;

    std::array<std::string, 6> leg_names = {"front_left", "middle_left", "rear_left", "rear_right", "middle_right", "front_right"};
    std::array<rclcpp::Publisher<hapthexa_msgs::msg::LegArgs>::SharedPtr, 6> present_leg_args_pubs;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HaptHexaServo>());
    rclcpp::shutdown();
    return 0;
}
