
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Geometry>

#include "hapthexa_msgs/msg/leg_position.hpp"
#include "hapthexa_msgs/msg/leg_args.hpp"

#include "hapthexa_msgs/msg/force_sensor.hpp"

#include "hapthexa_msgs/srv/get_leg_args.hpp"
#include "hapthexa_msgs/srv/get_leg_position.hpp"

#include "hapthexa_msgs/action/move_leg.hpp"
#include "hapthexa_msgs/action/move_leg_args.hpp"

using namespace std::chrono_literals;

enum joint_num
{
    coxa = 0,
    femur,
    tibia
};

class HaptHexaLeg : public rclcpp::Node
{
public:
    HaptHexaLeg() : Node("hapthexa_leg")
    {
        this->declare_parameter<double>("leg_install_angle", 0.0);
        this->get_parameter("leg_install_angle", leg_install_angle_);
        leg_position_sub_ = this->create_subscription<hapthexa_msgs::msg::LegPosition>(
            "leg_position", rclcpp::QoS(10), [this](const hapthexa_msgs::msg::LegPosition::SharedPtr leg_position) {
                // publish_args_by_position(leg_position->x, leg_position->y, leg_position->z);
                base_leg_position_ = *leg_position;
            });
        leg_args_pub_ = this->create_publisher<hapthexa_msgs::msg::LegArgs>("leg_args", rclcpp::QoS(10));

        present_leg_position_pub_ = this->create_publisher<hapthexa_msgs::msg::LegPosition>("present_leg_position", rclcpp::QoS(10));
        present_leg_args_sub_ = this->create_subscription<hapthexa_msgs::msg::LegArgs>(
            "present_leg_args", rclcpp::QoS(10),
            [this](const hapthexa_msgs::msg::LegArgs::SharedPtr present_leg_args) {
                present_leg_args_ = *present_leg_args;
                Eigen::Vector3d xyz = kinematics(present_leg_args->coxa_arg, present_leg_args->femur_arg, present_leg_args->tibia_arg);
                present_leg_position_.x = xyz(0);
                present_leg_position_.y = xyz(1);
                present_leg_position_.z = xyz(2);
                if (!present_leg_args_sub_called_)
                {
                    base_leg_position_ = present_leg_position_;
                    present_leg_args_sub_called_ = true;
                }
                present_leg_position_pub_->publish(present_leg_position_);
            });

        force_sensor_sub_ = this->create_subscription<hapthexa_msgs::msg::ForceSensor>("force_sensor", rclcpp::QoS(10),
                                                                                  [this](const hapthexa_msgs::msg::ForceSensor::SharedPtr force_sensor) {
                                                                                      force_sensor_z_detected_ = force_sensor->z;
                                                                                  });

        move_leg_action_server_ = rclcpp_action::create_server<hapthexa_msgs::action::MoveLeg>(
            this,
            "move_leg",
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const hapthexa_msgs::action::MoveLeg::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<hapthexa_msgs::action::MoveLeg>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<hapthexa_msgs::action::MoveLeg>> goal_handle) {
                if (move_leg_goal_handle_)
                {
                    move_leg_goal_handle_->abort(move_leg_result_);
                }
                RCLCPP_INFO(this->get_logger(), "move_leg action acepted");
                move_leg_goal_handle_ = goal_handle;
                move_leg_feedback_.reset(new hapthexa_msgs::action::MoveLeg::Feedback);
                move_leg_result_.reset(new hapthexa_msgs::action::MoveLeg::Result);

                relative_mode_ = goal_handle->get_goal()->relative_mode;
                if (!relative_mode_)
                {
                    diff_leg_position_.x = 0;
                    diff_leg_position_.y = 0;
                    diff_leg_position_.z = 0;
                }
                move_leg_present_xyz_ << present_leg_position_.x, present_leg_position_.y, present_leg_position_.z;
                move_leg_goal_xyz_ << goal_handle->get_goal()->x, goal_handle->get_goal()->y, goal_handle->get_goal()->z;
                if (relative_mode_)
                {
                    move_leg_present_xyz_(0) -= base_leg_position_.x;
                    move_leg_present_xyz_(1) -= base_leg_position_.y;
                    move_leg_present_xyz_(2) -= base_leg_position_.z;
                }
                Eigen::Vector3d diff = move_leg_goal_xyz_ - move_leg_present_xyz_;
                move_leg_unit_vector_ = diff / diff.norm();

                move_leg_args_called_ = false;
            });

        move_leg_args_action_server_ = rclcpp_action::create_server<hapthexa_msgs::action::MoveLegArgs>(
            this,
            "move_leg_args",
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const hapthexa_msgs::action::MoveLegArgs::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<hapthexa_msgs::action::MoveLegArgs>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<hapthexa_msgs::action::MoveLegArgs>> goal_handle) {
                if (move_leg_args_goal_handle_)
                {
                    move_leg_args_goal_handle_->abort(move_leg_args_result_);
                }
                RCLCPP_INFO(this->get_logger(), "move_leg_args action acepted");
                move_leg_args_goal_handle_ = goal_handle;
                move_leg_args_feedback_.reset(new hapthexa_msgs::action::MoveLegArgs::Feedback);
                move_leg_args_result_.reset(new hapthexa_msgs::action::MoveLegArgs::Result);

                move_leg_args_present_cft_[coxa] = present_leg_args_.coxa_arg;
                move_leg_args_present_cft_[femur] = present_leg_args_.femur_arg;
                move_leg_args_present_cft_[tibia] = present_leg_args_.tibia_arg;

                move_leg_args_goal_cft_[coxa] = goal_handle->get_goal()->coxa_arg;
                move_leg_args_goal_cft_[femur] = goal_handle->get_goal()->femur_arg;
                move_leg_args_goal_cft_[tibia] = goal_handle->get_goal()->tibia_arg;

                for (int i = 0; i < 3; i++)
                {
                    move_leg_args_finished_cft_[i] = false;
                }

                move_leg_args_called_ = true;
            });
    }
    void spin()
    {
        while (rclcpp::ok())
        {
            rclcpp::Rate rate(10ms);
            rate.sleep();
            rclcpp::spin_some(this->shared_from_this());
            if (move_leg_goal_handle_)
            {
                if (move_leg_goal_handle_->is_canceling())
                {
                    RCLCPP_INFO(this->get_logger(), "move_leg action canceled");
                    move_leg_goal_handle_->abort(move_leg_result_);
                    RCLCPP_INFO(this->get_logger(), "move_leg action cancel succeed");
                    move_leg_goal_handle_ = nullptr;
                }
                else
                {
                    move_leg_present_xyz_ += move_leg_velocity_ * 0.01 * move_leg_unit_vector_;
                    Eigen::Vector3d diff = move_leg_goal_xyz_ - move_leg_present_xyz_;

                    // move_leg_result_->x = move_leg_present_xyz_(0);
                    // move_leg_result_->y = move_leg_present_xyz_(1);
                    // move_leg_result_->z = move_leg_present_xyz_(2);


                    // present value
                    if (relative_mode_) {
                        move_leg_result_->x = diff_leg_position_.x;
                        move_leg_result_->y = diff_leg_position_.y;
                        move_leg_result_->z = diff_leg_position_.z;
                    } else {
                        move_leg_result_->x = present_leg_position_.x;
                        move_leg_result_->y = present_leg_position_.y;
                        move_leg_result_->z = present_leg_position_.z;
                    }

                    move_leg_feedback_->remaining = diff.norm();
                    move_leg_goal_handle_->publish_feedback(move_leg_feedback_);

                    if (diff.norm() < move_leg_velocity_ * 0.01 * 2.0)
                    {
                        
                        if (relative_mode_) {
                            diff_leg_position_.x = move_leg_goal_xyz_(0);
                            diff_leg_position_.y = move_leg_goal_xyz_(1);
                            diff_leg_position_.z = move_leg_goal_xyz_(2);
                        } else {
                            diff_leg_position_.x = 0;
                            diff_leg_position_.y = 0;
                            diff_leg_position_.z = 0;
                            base_leg_position_.x = move_leg_goal_xyz_(0);
                            base_leg_position_.y = move_leg_goal_xyz_(1);
                            base_leg_position_.z = move_leg_goal_xyz_(2);
                        }
                        publish_args_by_position(base_leg_position_.x + diff_leg_position_.x, base_leg_position_.y + diff_leg_position_.y, base_leg_position_.z + diff_leg_position_.z);
                        RCLCPP_INFO(this->get_logger(), "move_leg action succced");
                        move_leg_goal_handle_->succeed(move_leg_result_);
                        move_leg_goal_handle_ = nullptr;
                    }
                    else if (move_leg_goal_handle_->get_goal()->abort_if_forcesensor_z_detect_contact && force_sensor_z_detected_)
                    {
                        RCLCPP_INFO(this->get_logger(), "move_leg action abort because forcesensor z detect contact");
                        move_leg_goal_handle_->abort(move_leg_result_);
                        move_leg_goal_handle_ = nullptr;
                    }
                    else
                    {
                        if (relative_mode_) {
                            diff_leg_position_.x = move_leg_present_xyz_(0);
                            diff_leg_position_.y = move_leg_present_xyz_(1);
                            diff_leg_position_.z = move_leg_present_xyz_(2);
                        } else {
                            diff_leg_position_.x = 0;
                            diff_leg_position_.y = 0;
                            diff_leg_position_.z = 0;
                            base_leg_position_.x = move_leg_present_xyz_(0);
                            base_leg_position_.y = move_leg_present_xyz_(1);
                            base_leg_position_.z = move_leg_present_xyz_(2);
                        }
                        publish_args_by_position(base_leg_position_.x + diff_leg_position_.x, base_leg_position_.y + diff_leg_position_.y, base_leg_position_.z + diff_leg_position_.z);
                    }
                }
            }
            else if (move_leg_args_goal_handle_)
            {
                if (move_leg_args_goal_handle_->is_canceling())
                {
                    RCLCPP_INFO(this->get_logger(), "move_leg_args action canceled");
                    move_leg_args_goal_handle_->abort(move_leg_args_result_);
                    RCLCPP_INFO(this->get_logger(), "move_leg_args action cancel succeed");
                    move_leg_args_goal_handle_ = nullptr;
                }
                else
                {

                    for (int i = 0; i < 3; i++)
                    {
                        if (move_leg_args_finished_cft_[i])
                            continue;
                        move_leg_args_diff_cft_[i] = move_leg_args_goal_cft_[i] - move_leg_args_present_cft_[i];
                        if (fabs(move_leg_args_diff_cft_[i]) < move_leg_args_angular_velocity_ * 0.01 * 2)
                        {
                            move_leg_args_finished_cft_[i] = true;
                            move_leg_args_present_cft_[i] = move_leg_args_goal_cft_[i];
                            continue;
                        }
                        move_leg_args_present_cft_[i] += move_leg_args_angular_velocity_ * 0.01 * (move_leg_args_diff_cft_[i] > 0 ? 1.0 : -1.0);
                    }

                    move_leg_args_result_->coxa_arg = move_leg_args_present_cft_[coxa];
                    move_leg_args_result_->femur_arg = move_leg_args_present_cft_[femur];
                    move_leg_args_result_->tibia_arg = move_leg_args_present_cft_[tibia];

                    move_leg_args_feedback_->remaining = fabs(move_leg_args_diff_cft_[coxa]) + fabs(move_leg_args_diff_cft_[femur]) + fabs(move_leg_args_diff_cft_[tibia]);
                    move_leg_args_goal_handle_->publish_feedback(move_leg_args_feedback_);

                    if (move_leg_args_finished_cft_[coxa] && move_leg_args_finished_cft_[femur] && move_leg_args_finished_cft_[tibia])
                    {
                        Eigen::Vector3d base;
                        base = kinematics(move_leg_args_goal_cft_[0], move_leg_args_goal_cft_[1], move_leg_args_goal_cft_[2]);
                        diff_leg_position_.x = 0;
                        diff_leg_position_.y = 0;
                        diff_leg_position_.z = 0;
                        base_leg_position_.x = base(0);
                        base_leg_position_.y = base(1);
                        base_leg_position_.z = base(2);
                        publish_args(move_leg_args_goal_cft_[0], move_leg_args_goal_cft_[1], move_leg_args_goal_cft_[2]);
                        RCLCPP_INFO(this->get_logger(), "move_leg_args action succced");
                        move_leg_args_goal_handle_->succeed(move_leg_args_result_);
                        move_leg_args_goal_handle_ = nullptr;
                    }
                    else
                    {
                        publish_args(move_leg_args_present_cft_[0], move_leg_args_present_cft_[1], move_leg_args_present_cft_[2]);
                    }
                }
            }
            else
            {
                if (present_leg_args_sub_called_ && !move_leg_args_called_)
                {
                    publish_args_by_position(base_leg_position_.x + diff_leg_position_.x, base_leg_position_.y + diff_leg_position_.y, base_leg_position_.z + diff_leg_position_.z);
                }
            }
        }
    }

private:
    Eigen::Vector3d kinematics(double coxa, double femur, double tibia)
    {
        Eigen::Vector3d ret;
        Eigen::Matrix3d rot;

        double l1 = 4.5, l2 = 10.0, l3 = 15.0;
        double r = l1 + l2 * cos(femur) + l3 * cos(femur + tibia);
        Eigen::Vector3d xyz(r * cos(coxa), r * sin(coxa), l2 * sin(femur) + l3 * sin(femur + tibia));
        rot = Eigen::AngleAxisd(leg_install_angle_, Eigen::Vector3d::UnitZ());
        ret = rot * xyz;
        return ret;
    }
    void publish_args_by_position(double _x, double _y, double _z) const
    {
        Eigen::Matrix3d rot;
        Eigen::Vector3d toes_local;
        Eigen::Vector3d toes(_x, _y, _z);

        rot = Eigen::AngleAxisd(-leg_install_angle_, Eigen::Vector3d::UnitZ());
        toes_local = rot * toes;

        double l1 = 4.5, l2 = 10.0, l3 = 15.0;
        double x = toes_local(0), y = toes_local(1), z = toes_local(2);
        double r = sqrt(x * x + y * y) - l1;
        double tmp = acos((z * z + r * r + l2 * l2 - l3 * l3) / (2.0 * l2 * sqrt(z * z + r * r)));

        double coxa_arg = atan2(y, x);
        double femur_arg = atan2(z, r) + (tmp > 0 ? tmp : -tmp);
        double tibia_arg = atan2((z - l2 * sin(femur_arg)), (r - l2 * cos(femur_arg))) - femur_arg;

        publish_args(coxa_arg, femur_arg, tibia_arg);
    }
    void publish_args(double coxa, double femur, double tibia) const
    {
        auto leg_args = hapthexa_msgs::msg::LegArgs();

        leg_args.coxa_arg = coxa;
        leg_args.femur_arg = femur < -M_PI/2.0 ? -M_PI/2.0 : femur > M_PI/2.0 ? M_PI/2.0 : femur;
        leg_args.tibia_arg = tibia < -140.0/180.0*M_PI ? -140.0/180.0*M_PI : tibia > 140.0/180.0*M_PI ? 140.0/180.0*M_PI : tibia;

        leg_args_pub_->publish(leg_args);
    }
    rclcpp::Subscription<hapthexa_msgs::msg::LegPosition>::SharedPtr leg_position_sub_;
    rclcpp::Publisher<hapthexa_msgs::msg::LegArgs>::SharedPtr leg_args_pub_;
    hapthexa_msgs::msg::LegPosition base_leg_position_ = hapthexa_msgs::msg::LegPosition();
    hapthexa_msgs::msg::LegPosition diff_leg_position_ = hapthexa_msgs::msg::LegPosition();
    bool relative_mode_ = false;

    rclcpp::Subscription<hapthexa_msgs::msg::LegArgs>::SharedPtr present_leg_args_sub_;
    rclcpp::Publisher<hapthexa_msgs::msg::LegPosition>::SharedPtr present_leg_position_pub_;
    hapthexa_msgs::msg::LegArgs present_leg_args_ = hapthexa_msgs::msg::LegArgs();
    hapthexa_msgs::msg::LegPosition present_leg_position_ = hapthexa_msgs::msg::LegPosition();
    bool present_leg_args_sub_called_ = false;

    rclcpp::Subscription<hapthexa_msgs::msg::ForceSensor>::SharedPtr force_sensor_sub_;
    double force_sensor_z_detected_ = false;

    rclcpp_action::Server<hapthexa_msgs::action::MoveLeg>::SharedPtr move_leg_action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<hapthexa_msgs::action::MoveLeg>> move_leg_goal_handle_;
    std::shared_ptr<hapthexa_msgs::action::MoveLeg::Feedback> move_leg_feedback_;
    std::shared_ptr<hapthexa_msgs::action::MoveLeg::Result> move_leg_result_;
    Eigen::Vector3d move_leg_present_xyz_;
    Eigen::Vector3d move_leg_goal_xyz_;
    Eigen::Vector3d move_leg_unit_vector_;

    rclcpp_action::Server<hapthexa_msgs::action::MoveLegArgs>::SharedPtr move_leg_args_action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<hapthexa_msgs::action::MoveLegArgs>> move_leg_args_goal_handle_;
    std::shared_ptr<hapthexa_msgs::action::MoveLegArgs::Feedback> move_leg_args_feedback_;
    std::shared_ptr<hapthexa_msgs::action::MoveLegArgs::Result> move_leg_args_result_;
    double move_leg_args_goal_cft_[3];
    double move_leg_args_present_cft_[3];
    double move_leg_args_diff_cft_[3];
    bool move_leg_args_finished_cft_[3];
    bool move_leg_args_called_ = true;

    double move_leg_velocity_ = 5.0;
    double move_leg_args_angular_velocity_ = M_PI / 6.0;
    double leg_install_angle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HaptHexaLeg>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}