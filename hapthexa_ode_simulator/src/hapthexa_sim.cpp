#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>

#include "hapthexa_msgs/msg/leg_args.hpp"
#include "hapthexa_msgs/srv/get_leg_args.hpp"
#include "hapthexa_msgs/msg/force_sensor.hpp"
#include "hapthexa_msgs/msg/attitude.hpp"

#include "ODE.h"
#include <cinttypes>
#include <eigen3/Eigen/Geometry>

using namespace std::chrono_literals;
rclcpp::WallRate rate(5ms);

#define LINK_NUM 4 // 全リンク数 (total number of links)
#define JT_NUM 3   // 全ジョイント数 (total number of joints)
#define LEG_NUM 6  // 全脚数 (total number of legs)

#define RSF_X_NUM 5
#define RSF_Y_NUM 10

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif

// #define BOX
// #define RSF

struct leg_state
{
    // 各リンク長
    const double coxa_length = 4, femur_length = 10, tibia_length = 12.5;
    // 脚の接続向き
    double angle;      // 脚の角度
    double root_angle; // 脚の付け根の角度

    // 各関節角度の一時変数
    int32_t coxa_encoder_val, femur_encoder_val, tibia_encoder_val; //value that order to encoder between 0 - 4048

    // [output]現在の脚先座標
    Eigen::Vector3d toes_current;
    double coxa_arg_current, femur_arg_current, tibia_arg_current;

    // [output]
    double force_x, force_y, force_z;

    // [input]
    Eigen::Vector3d toes;
    double coxa_arg, femur_arg, tibia_arg; // -pi to pi
};

typedef struct
{
    dBodyID body;
    dGeomID geom;
    dJointID joint;
    dJointFeedback fb;
    dReal m, r, x, y, z; // 質量(weight)，半径(radius)，位置(positin:x,y,z)
} MyLink;

enum leg_num
{
    front_left = 0,
    middle_left,
    rear_left,
    rear_right,
    middle_right,
    front_right,
};

enum joint_num
{
    coxa = 0,
    femur,
    tibia
};

dWorldID world;
dSpaceID space;

dGeomID slope;

dGeomID rsf[RSF_Y_NUM][RSF_X_NUM];

MyLink leg[LEG_NUM][LINK_NUM], torso; // 脚(leg)，胴体(torso)
const dReal x00 = 0.04 * sqrt(3.0), y00 = 0.08;

// dReal  THETA[LEG_NUM][LINK_NUM] = {{M_PI_4,M_PI_4,M_PI_4},{0},{0},{0}}; // 目標角度(target angle)
dReal THETA[LEG_NUM][LINK_NUM] = {{0.0}};

dReal torso_m = 10.0;                                  // 胴体の質量 (weight of torso)
const dReal lx = 2.0 * x00, ly = 2.0 * y00, lz = 0.05; // 0.05;         // body sides
const dReal SX = 1, SY = 0, SZ = 0.26;                 // 胴体重心の初期位置(initial positon of COG)

const dReal l1 = 0.04, l2 = 0.1, l3 = 0.04, l4 = 0.12;      // リンク長 0.25 (lenth of links)
const dReal l1m = 0.005, l2m = 0.2, l3m = 0.08, l4m = 0.12; // リンクの質量 (weight of links)
const dReal r1 = 0.016, r2 = 0.02, r3 = 0.02, r4 = 0.012;   // leg radius

dReal r[LINK_NUM] = {r1, r2, r3, r4};          // リンクの半径 (radius of links)
dReal length[LINK_NUM] = {l1, l2, l3, l4};     // リンクの長さ (length of links)
dReal weight[LINK_NUM] = {l1m, l2m, l3m, l4m}; // リンクの質量 (weight of links)
dReal axis_x[LEG_NUM][LINK_NUM] = {{0, 0, 0}, {0, 1, 1}, {0, 0, 0}, {0, 0, 0}, {0, 1, 1}, {0, 0, 0}};
dReal axis_y[LEG_NUM][LINK_NUM] = {{0, 1, 1}, {0, 0, 0}, {0, 1, 1}, {0, 1, 1}, {0, 0, 0}, {0, 1, 1}};
dReal axis_z[LEG_NUM][LINK_NUM] = {{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}};

double roll = 0.0, pitch = 0.0;

double slope_angle = -0.3;

leg_state leg_s[6];

void init()
{
    
    leg_s[front_left].angle = M_PI / 6.0;
    leg_s[middle_left].angle = M_PI / 2.0;
    leg_s[rear_left].angle = M_PI * 5.0 / 6.0;
    leg_s[front_right].angle = -M_PI / 6.0;
    leg_s[middle_right].angle = -M_PI / 2.0;
    leg_s[rear_right].angle = -M_PI * 5.0 / 6.0;

    leg_s[front_left].root_angle = leg_s[front_right].root_angle = 0.0;
    leg_s[middle_left].root_angle = M_PI / 2.0;
    leg_s[middle_right].root_angle = -M_PI / 2.0;
    leg_s[rear_left].root_angle = leg_s[rear_right].root_angle = -M_PI;
    // 胴体の生成 (crate a torso)
    // dsSetColor(1.0,1.0,1.0);
    dMass mass;
    torso.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, torso_m, lx, ly, lz);
    dBodySetMass(torso.body, &mass);
    torso.geom = dCreateBox(space, lx, ly, lz);
    dGeomSetBody(torso.geom, torso.body);
    dBodySetPosition(torso.body, SX, SY, SZ);

    // 脚の生成 (create 4 legs)
    dReal side = 0.08;
    dReal position[LINK_NUM] = {l1 / 2.0, l1 + l2 / 2.0, l1 + l2 + l3 / 2.0, l1 + l2 + l3 + l4 / 2.0};
    dReal joint_position[LINK_NUM] = {0.0, l1, l1 + l2, l1 + l2 + l3};
    dMatrix3 R; // 回転行列
    for (int i = 0; i < LEG_NUM; i++)
    {
        for (int j = 0; j < LINK_NUM; j++)
        {
            leg[i][j].body = dBodyCreate(world);
            Eigen::Matrix3d rot;
            rot = Eigen::AngleAxisd(leg_s[i].root_angle, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY());
            for (int k = 0; k < 3; k++)
            {
                for (int l = 0; l < 3; l++)
                {
                    R[4 * k + l] = rot(k, l);
                }
            }
            dBodySetRotation(leg[i][j].body, R);
            dBodySetPosition(
                leg[i][j].body,
                SX + side * cos(leg_s[i].angle) + position[j] * cos(leg_s[i].root_angle),
                SY + side * sin(leg_s[i].angle) + position[j] * sin(leg_s[i].root_angle),
                SZ);
            dMassSetZero(&mass);
            dMassSetCapsuleTotal(&mass, weight[j], 3, r[j], length[j]);
            dBodySetMass(leg[i][j].body, &mass);
            leg[i][j].geom = dCreateCapsule(space, r[j], length[j]);
            dGeomSetBody(leg[i][j].geom, leg[i][j].body);
        }
    }

    // ジョイントの生成とリンクへの取り付け (create links and attach legs to the torso)
    for (int i = 0; i < LEG_NUM; i++)
    {
        for (int j = 0; j < LINK_NUM; j++)
        {
            if (j == LINK_NUM - 1) // 床反力センサ
            {
                leg[i][j].joint = dJointCreateFixed(world, 0);
                dJointAttach(leg[i][j].joint, leg[i][j - 1].body, leg[i][j].body);
                dJointSetFixed(leg[i][j].joint);
                // ジョイントのフィードバックの設定
                dJointSetFeedback(leg[i][j].joint, &(leg[i][j].fb));
            }
            else
            {
                leg[i][j].joint = dJointCreateHinge(world, 0);
                if (j == 0)
                    dJointAttach(leg[i][j].joint, torso.body, leg[i][j].body);
                else
                    dJointAttach(leg[i][j].joint, leg[i][j - 1].body, leg[i][j].body);
                dJointSetHingeAnchor(
                    leg[i][j].joint,
                    SX + side * cos(leg_s[i].angle) + joint_position[j] * cos(leg_s[i].root_angle),
                    SY + side * sin(leg_s[i].angle) + joint_position[j] * sin(leg_s[i].root_angle),
                    SZ);
                dJointSetHingeAxis(leg[i][j].joint, axis_x[i][j], axis_y[i][j], axis_z[i][j]);
            }
        }
    }

    // IC
    // leg_state leg_ic[6];
    // for (int i = 0; i < 6; i++)
    // {
    //     switch (i)
    //     {
    //     case front_right:
    //     case rear_left:
    //         leg_ic[i].coxa_arg = M_PI / 2.0;
    //         break;
    //     case rear_right:
    //     case front_left:
    //         leg_ic[i].coxa_arg = -M_PI / 2.0;
    //         break;
    //     default:
    //         leg_ic[i].coxa_arg = 0.0;
    //         break;
    //     }
    //     leg_ic[i].femur_arg = M_PI / 2.0;
    //     leg_ic[i].tibia_arg = M_PI/2.0 - (i%2 ? M_PI/6 : 0);
    // }

    // for (int i = 0; i < 50; i++) {
    //     moveAllLegs(leg_ic, true);
    // }

#ifdef BOX
    //box
    slope = dCreateBox(space, 2, 1.5, 1);
    dRFromAxisAndAngle(R, 0, 1, 0, slope_angle);
    dGeomSetPosition(slope, 2, 0, -0.34);
    dGeomSetRotation(slope, R);
#endif

#ifdef RSF
    //rsf
    for (int y = 0; y < RSF_Y_NUM; y++)
    {
        for (int x = 0; x < RSF_X_NUM; x++)
        {
            // double h = (rand() % 5) * 0.01 + 0.02;
            double h = 0.04;
            rsf[y][x] = dCreateBox(space, 0.1, 0.1, h);
            dGeomSetPosition(rsf[y][x], 0.1 * x + 0.1 / 2.0 + 1.25, 0.1 * y + 0.1 / 2.0 - 0.5, h / 2.0);
            // double h = (rand() % 5) * 0.03 + 0.05;
            // rsf[y][x] = dCreateBox(space, 0.1, 0.1, h);
            // dGeomSetPosition(rsf[y][x], 0.1 * x + 0.1 / 2.0, 0.1 * y + 0.1 / 2.0 - 0.5, h / 2.0);
        }
    }
#endif
}

void loop(int pause, rclcpp::Node::SharedPtr node) //rclcpp::Node::SharedPtr node
{
    (void)pause;
    if (!rclcpp::ok())
    {
        dsStop();
        return;
    }
    rclcpp::spin_some(node);
    rate.sleep();

    for (int i = 0; i < LEG_NUM; i++)
    {
        for (int j = 0; j < LINK_NUM; j++)
        {
            if (THETA[i][j] > M_PI)
                THETA[i][j] = M_PI;
            if (THETA[i][j] < -M_PI)
                THETA[i][j] = -M_PI;
            if (std::isnan(THETA[i][j]))
                THETA[i][j] = 0;
            // ジョイントのフィードバック
            if (j == LINK_NUM - 1)
            {
                dJointGetFeedback(leg[i][j].joint);

                // ベクトルを足先に合わせ回転
                const dReal *bodyRotation = dBodyGetRotation(leg[i][j].body);
                double ret[3] = {};
                for (int k = 0; k < 3; k++)
                {
                    for (int l = 0; l < 3; l++)
                    {
                        ret[k] += bodyRotation[4 * k + l] * leg[i][j].fb.f1[l];
                    }
                }
                leg_s[i].force_x = ret[0];
                leg_s[i].force_y = ret[1];
                leg_s[i].force_z = ret[2];
                // if (i == front_left)
                // {
                    // std::cout << std::fixed << std::setprecision(2);
                    // std::cout << ret[0] << "\t" << ret[1] << "\t" << ret[2];
                // }
                // std::cout << (ret[2]>2) << "\t";
            }
        }
    }
    // std::cout << std::endl;

    //P制御
    dReal kp = 20.0, fMax = 3.4;
    for (int i = 0; i < LEG_NUM; i++)
    {
        for (int j = 0; j < LINK_NUM - 1; j++)
        {
            dReal tmp = dJointGetHingeAngle(leg[i][j].joint);
            dReal diff = THETA[i][j] - tmp;
            dReal u = kp * diff;
            dJointSetHingeParam(leg[i][j].joint, dParamVel, u);
            dJointSetHingeParam(leg[i][j].joint, dParamFMax, fMax);
        }
    }

    dReal r, length;
    dVector3 sides;

    // 胴体の描画
    dsSetColor(1.3, 1.3, 1.3);
    dGeomBoxGetLengths(torso.geom, sides);
    dsDrawBox(dBodyGetPosition(torso.body),
              dBodyGetRotation(torso.body), sides);

    // 脚の描画
    for (int i = 0; i < LEG_NUM; i++)
    {
        for (int j = 0; j < LINK_NUM; j++)
        {
            dGeomCapsuleGetParams(leg[i][j].geom, &r, &length);
            if (j == 0)
                dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
                              dBodyGetRotation(leg[i][j].body), 0.5 * length, 1.2 * r);
            else
                dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
                              dBodyGetRotation(leg[i][j].body), length, r);
        }
    }
    dsSetColor(0xcd / 255.0, 0x85 / 255.0, 0x3f / 255.0);
#ifdef BOX
    // 坂の描画
    dVector3 ss;
    dGeomBoxGetLengths(slope, ss);
    dsDrawBox(dGeomGetPosition(slope), dGeomGetRotation(slope), ss);
#endif

#ifdef RSF
    //rsf
    for (int y = 0; y < RSF_Y_NUM; y++)
    {
        for (int x = 0; x < RSF_X_NUM; x++)
        {
            dVector3 ss;
            dGeomBoxGetLengths(rsf[y][x], ss);
            dsDrawBox(dGeomGetPosition(rsf[y][x]), dGeomGetRotation(rsf[y][x]), ss);
        }
    }
#endif

    // IMUエミュレーション

    const dReal *bodyRotation = dBodyGetRotation(torso.body);
    roll = atan2(bodyRotation[9], bodyRotation[10]);
    pitch = -atan2(bodyRotation[8], sqrt(bodyRotation[9] * bodyRotation[9] + bodyRotation[10] * bodyRotation[10]));
    // printf("r=%f\t, p=%f\n", roll, pitch);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hapthexa_sim");
    auto front_left_sub = node->create_subscription<hapthexa_msgs::msg::LegArgs>(
        "hapthexa/leg/front_left/leg_args",
        rclcpp::QoS(10),
        [&](const hapthexa_msgs::msg::LegArgs::SharedPtr msg) -> void {
            THETA[front_left][coxa] = -msg->coxa_arg;
            THETA[front_left][femur] = msg->femur_arg;
            THETA[front_left][tibia] = msg->tibia_arg;
        });
    auto middle_left_sub = node->create_subscription<hapthexa_msgs::msg::LegArgs>(
        "hapthexa/leg/middle_left/leg_args",
        rclcpp::QoS(10),
        [&](const hapthexa_msgs::msg::LegArgs::SharedPtr msg) -> void {
            THETA[middle_left][coxa] = -msg->coxa_arg;
            THETA[middle_left][femur] = -msg->femur_arg;
            THETA[middle_left][tibia] = -msg->tibia_arg;
        });
    auto rear_left_sub = node->create_subscription<hapthexa_msgs::msg::LegArgs>(
        "hapthexa/leg/rear_left/leg_args",
        rclcpp::QoS(10),
        [&](const hapthexa_msgs::msg::LegArgs::SharedPtr msg) -> void {
            THETA[rear_left][coxa] = -msg->coxa_arg;
            THETA[rear_left][femur] = -msg->femur_arg;
            THETA[rear_left][tibia] = -msg->tibia_arg;
        });
    auto rear_right_sub = node->create_subscription<hapthexa_msgs::msg::LegArgs>(
        "hapthexa/leg/rear_right/leg_args",
        rclcpp::QoS(10),
        [&](const hapthexa_msgs::msg::LegArgs::SharedPtr msg) -> void {
            THETA[rear_right][coxa] = -msg->coxa_arg;
            THETA[rear_right][femur] = -msg->femur_arg;
            THETA[rear_right][tibia] = -msg->tibia_arg;
        });
    auto middle_right_sub = node->create_subscription<hapthexa_msgs::msg::LegArgs>(
        "hapthexa/leg/middle_right/leg_args",
        rclcpp::QoS(10),
        [&](const hapthexa_msgs::msg::LegArgs::SharedPtr msg) -> void {
            THETA[middle_right][coxa] = -msg->coxa_arg;
            THETA[middle_right][femur] = msg->femur_arg;
            THETA[middle_right][tibia] = msg->tibia_arg;
        });
    auto front_right_sub = node->create_subscription<hapthexa_msgs::msg::LegArgs>(
        "hapthexa/leg/front_right/leg_args",
        rclcpp::QoS(10),
        [&](const hapthexa_msgs::msg::LegArgs::SharedPtr msg) -> void {
            THETA[front_right][coxa] = -msg->coxa_arg;
            THETA[front_right][femur] = msg->femur_arg;
            THETA[front_right][tibia] = msg->tibia_arg;
        });

    std::array<std::string, 6> leg_names = {"front_left", "middle_left", "rear_left", "rear_right", "middle_right", "front_right"};
    std::array<rclcpp::Publisher<hapthexa_msgs::msg::LegArgs>::SharedPtr, 6> present_leg_args_pubs;
    for (int i = 0; i < 6; i++)
    {
        present_leg_args_pubs[i] = node->create_publisher<hapthexa_msgs::msg::LegArgs>("hapthexa/leg/" + leg_names[i] + "/present_leg_args", rclcpp::QoS(10));
    }
    std::array<rclcpp::Publisher<hapthexa_msgs::msg::ForceSensor>::SharedPtr, 6> force_sensor_pubs;
    for (int i = 0; i < 6; i++)
    {
        force_sensor_pubs[i] = node->create_publisher<hapthexa_msgs::msg::ForceSensor>("hapthexa/leg/" + leg_names[i] + "/force_sensor", rclcpp::QoS(10));
    }

    auto attitude_publisher = node->create_publisher<hapthexa_msgs::msg::Attitude>("hapthexa/attitude", rclcpp::QoS(10));

    auto timer = node->create_wall_timer(10ms, [&]() {
        for (int i = 0; i < LEG_NUM; i++)
        {
            auto attitude = hapthexa_msgs::msg::Attitude();
            attitude.roll = roll;
            attitude.pitch = pitch;
            attitude_publisher->publish(attitude);
            auto force_sensor = hapthexa_msgs::msg::ForceSensor();
            force_sensor.z = leg_s[i].force_z > 10;
            force_sensor_pubs[i]->publish(force_sensor);
            auto leg_args = hapthexa_msgs::msg::LegArgs();
            for (int j = 0; j < LINK_NUM - 1; j++)
            {
                double arg = dJointGetHingeAngle(leg[i][j].joint);
                if ((i == middle_left || i == rear_left || i == rear_right) || (j == 0))
                {
                    arg *= -1.0;
                }
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

    

    ODE ode(std::bind(loop, std::placeholders::_1, node));
    world = ode.getWorld();
    space = ode.getSpace();
    init();
    ode.spin();
    rclcpp::shutdown();
    return 0;
}
