#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/VFR_HUD.h>
#include <std_msgs/Float64.h>
#include <tracker_msgs/TrackerCmd.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
// #include <sensor_msgs/BatteryState.h>
// #include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>
#include <fast_model/fast_model.h>

enum px4_state{
    WAITE_FOR_ARM,
    WAITE_FOR_OFFBOARD,
    OK
};

enum DEFAULT_RC_CHANNEL{
    ROLL,
    PITCH,
    THROTTLE,
    YAW,
    MODE,
    GEAR,
    NONE,
    REBOOT
};

class FastModelInterface : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        void all_in_one_timer_CB(const ros::TimerEvent&);
        // void cmd_CB(const tracker_msgs::TrackerCmd::ConstPtr &msg);
        void cmd_CB(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
        bool setModeCB(mavros_msgs::SetMode::Request &req, mavros_msgs::SetMode::Response &res);
        bool armCB(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res);

        ros::Publisher mav_pos_pub_, mav_vel_pub_, odom_pub_, state_pub_, imu_pub_, rc_in_pub_, extended_state_pub_, battery_status_pub_;

        ros::Subscriber ctrl_cmd_sub_;

        ros::ServiceServer set_mode_srv_, arm_srv_;

        fast_model::FastRotorsModel model_;

        ros::Timer AllinOne_timer_;

        px4_state cur_state_{px4_state::WAITE_FOR_ARM};
};

void FastModelInterface::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    /* publish */
    mav_pos_pub_ = priv_nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10);
    mav_vel_pub_ = priv_nh.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10);
    odom_pub_ = priv_nh.advertise<nav_msgs::Odometry>("odom_enu", 10);
    state_pub_ = priv_nh.advertise<mavros_msgs::State>("mavros/state", 10);
    rc_in_pub_ = priv_nh.advertise<mavros_msgs::RCIn>("mavros/rc/in", 10);
    extended_state_pub_ = priv_nh.advertise<mavros_msgs::ExtendedState>("mavros/extended_state", 10);
    battery_status_pub_ = priv_nh.advertise<sensor_msgs::BatteryState>("mavros/battery", 10);
    imu_pub_ = priv_nh.advertise<sensor_msgs::Imu>("enu_imu/data_raw", 10);

    /* subscribe */
    ctrl_cmd_sub_ = priv_nh.subscribe("mavros/setpoint_raw/attitude", 10, &FastModelInterface::cmd_CB, this);

    /* service server */
    set_mode_srv_ = priv_nh.advertiseService("mavros/set_mode", &FastModelInterface::setModeCB, this);
    arm_srv_ = priv_nh.advertiseService("mavros/cmd/arming", &FastModelInterface::armCB, this);

    AllinOne_timer_ = priv_nh.createTimer(ros::Duration(0.01), &FastModelInterface::all_in_one_timer_CB, this);

    double init_px, init_py, init_pz, init_yaw, init_V;
    priv_nh.param<double>("init_position/x", init_px, 0.0);
    priv_nh.param<double>("init_position/y", init_py, 0.0);
    priv_nh.param<double>("init_position/z", init_pz, 500.0);
    priv_nh.param<double>("init_position/yaw", init_yaw, 0.0);
    priv_nh.param<double>("init_flight_speed", init_V, 20.0);

    fast_model::FastRotorsModel::State init_state;

    init_state.pos = {init_px, init_py, init_pz};
    init_state.vel = {0.0, 0.0, 0.0};
    init_state.acc = {0.0, 0.0, 0.0};
    init_state.yaw = init_yaw;
    init_state.roll = 0.0;
    init_state.pitch = 0.0;
    fast_model::FastRotorsModel::Control init_u;
    init_u.thrust = 0.0;
    init_u.att_q.w() = 1.0;
    init_u.att_q.x() = 0.0;
    init_u.att_q.y() = 0.0;
    init_u.att_q.z() = 0.0;
    model_.setInitState(init_state);
    model_.setCtrl(init_u);
}

// void FastModelInterface::cmd_CB(const tracker_msgs::TrackerCmd::ConstPtr &msg) {
void FastModelInterface::cmd_CB(const mavros_msgs::AttitudeTarget::ConstPtr &msg) {
    fast_model::FastRotorsModel::Control u;
    u.att_q.w() = msg->orientation.w;
    u.att_q.x() = msg->orientation.x;
    u.att_q.y() = msg->orientation.y;
    u.att_q.z() = msg->orientation.z;
    u.thrust = msg->thrust;
    if (cur_state_ == px4_state::OK)
        model_.setCtrl(u);
}

bool FastModelInterface::setModeCB(mavros_msgs::SetMode::Request &req, mavros_msgs::SetMode::Response &res) {
    if (cur_state_ == px4_state::WAITE_FOR_OFFBOARD) {
        NODELET_INFO_STREAM("set offboard mode");
        cur_state_ = px4_state::WAITE_FOR_ARM;
        return true;
    }
    return false;
}

bool FastModelInterface::armCB(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res) {
    if (cur_state_ == px4_state::WAITE_FOR_ARM && req.value) {
        NODELET_INFO_STREAM("arm the drone");
        res.success = true;
        cur_state_ = px4_state::OK;
        return true;
    } else if (cur_state_ == px4_state::OK && !req.value) {
        NODELET_INFO_STREAM("disarm the drone");
        res.success = true;
        cur_state_ = px4_state::WAITE_FOR_ARM;
        return true;
    }
    return false;
}

void FastModelInterface::all_in_one_timer_CB(const ros::TimerEvent&) {
    static ros::Time last_time = ros::Time::now();
    double dt = (ros::Time::now() - last_time).toSec();
    dt = std::max(std::min(dt, 0.1), 0.001);
    last_time = ros::Time::now();
    ROS_INFO_THROTTLE(10.0, "dt: %.4f", dt);
    model_.propagate(dt);
    nav_msgs::Odometry odom = model_.getOdomfromState();
    odom_pub_.publish(odom);

    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header = odom.header;
    pos_msg.pose = odom.pose.pose;
    mav_pos_pub_.publish(pos_msg);

    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header = odom.header;
    vel_msg.twist = odom.twist.twist;
    mav_vel_pub_.publish(vel_msg);

    sensor_msgs::Imu imu_msg = model_.getImufromState();
    imu_pub_.publish(imu_msg);

    /* rc in update */
    mavros_msgs::RCIn rc_in_msg;
    rc_in_msg.header = odom.header;
    rc_in_msg.channels = std::vector<uint16_t>(8, 1500);
    rc_in_msg.channels[DEFAULT_RC_CHANNEL::MODE] = 1900;
    rc_in_msg.channels[DEFAULT_RC_CHANNEL::GEAR] = 1900;
    rc_in_msg.channels[DEFAULT_RC_CHANNEL::NONE] = 100;
    rc_in_msg.channels[DEFAULT_RC_CHANNEL::REBOOT] = 400;
    rc_in_pub_.publish(rc_in_msg);

    /* state update */
    mavros_msgs::State state_msg;
    state_msg.header = odom.header;
    state_msg.connected = true;
    state_msg.armed = cur_state_ > px4_state::WAITE_FOR_ARM;
    state_msg.mode = mavros_msgs::State::MODE_PX4_OFFBOARD;
    state_pub_.publish(state_msg);

    /* extended state update */
    mavros_msgs::ExtendedState ex_state_msg;
    ex_state_msg.header = odom.header;
    ex_state_msg.landed_state = model_.onGroundCheck() ? mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND : mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    extended_state_pub_.publish(ex_state_msg);

    /* battery state update */
    sensor_msgs::BatteryState bat_state_msg;
    bat_state_msg.header = odom.header;
    bat_state_msg.cell_voltage = std::vector<float>(4, 4.0f);
    bat_state_msg.percentage = 0.8;
    battery_status_pub_.publish(bat_state_msg);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(FastModelInterface, nodelet::Nodelet);