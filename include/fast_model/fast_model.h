#ifndef FAST_ROTORS_MODEL_H_
#define FAST_ROTORS_MODEL_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <fast_model/AlphaFilter.hpp>
#include <fast_model/AlphaRadFilter.hpp>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>

#define CONSTANTS_ONE_G 9.80665f
#define HOVER_THROTTLE 0.4

namespace fast_model
{
    class FastRotorsModel {
        public:
            typedef struct {
                Eigen::Vector3d pos;
                Eigen::Vector3d vel;
                Eigen::Vector3d acc;
                double roll;
                double pitch;
                double yaw;
            } State;
            
            typedef struct {
                Eigen::Quaterniond att_q;
                double thrust;
            } Control;

            FastRotorsModel() {
                thrust_filter_.setParameters(0.01, 0.1);
                roll_filter_.setParameters(0.01, 0.08);
                pitch_filter_.setParameters(0.01, 0.08);
                yaw_filter_.setParameters(0.01, 0.4);
                scale_acc_thrust_ = 1.5f * CONSTANTS_ONE_G / HOVER_THROTTLE;
            };
            ~FastRotorsModel();

            void setInitState(State s) {
                if (!state_set_) {
                    roll_filter_.reset(s.roll);
                    pitch_filter_.reset(s.pitch);
                    yaw_filter_.reset(s.yaw);
                    thrust_filter_.reset(0.0);
                    state_set_ = true;
                }
                s_ = s;
            };

            void setCtrl(Control u) {
                if (!ctrl_set_) {
                    ctrl_set_ = true;
                }
                u_.att_q = u.att_q;
                u_.thrust = constraint(u.thrust, 0.0, 1.0);
            }

            void propagate(double dt) {
                /* propagation */
                Eigen::Vector3d euler_att;
                get_euler_from_q(euler_att, u_.att_q);
                s_.roll  = roll_filter_.update(euler_att(0));
                s_.pitch = pitch_filter_.update(euler_att(1));
                s_.yaw   = yaw_filter_.update(euler_att(2));
                double throttle = thrust_filter_.update(u_.thrust);
                double Ta = constraint((throttle - HOVER_THROTTLE) * scale_acc_thrust_ + CONSTANTS_ONE_G, 0.0, 10000.0);
                Eigen::Vector3d body_T_z(0.0, 0.0, Ta);
                Eigen::Matrix3d w_R_b;
                get_dcm_from_euler(w_R_b, Eigen::Vector3d(s_.roll, s_.pitch, s_.yaw));
                s_.acc = w_R_b * body_T_z - Eigen::Vector3d(0.0, 0.0, CONSTANTS_ONE_G);
                s_.vel = s_.vel + s_.acc * dt;
                s_.pos = s_.pos + s_.vel * dt + s_.acc * dt * dt / 2.0;
                /* ground effect*/
                if (s_.pos(2) < 0.0) {
                    s_.vel(2) = s_.vel(2) < 0.0 ? 0.0 : s_.vel(2);
                    s_.acc(2) = s_.acc(2) < 0.0 ? 0.0 : s_.acc(2);
                    s_.pos(2) = 0.0;
                }
                on_ground_ = s_.pos(2) > 0.05 ? false: true;
            };

            bool onGroundCheck() {return on_ground_;};

            // void precise_propagate(double dt) {
            //     acc_filter_.update(pu_.thrust*scale_acc_thrust_);
            // }
            sensor_msgs::Imu getImufromState() {
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "world";

                Eigen::Matrix3d w_R_b;
                get_dcm_from_euler(w_R_b, Eigen::Vector3d(s_.roll, s_.pitch, s_.yaw));
                Eigen::Quaterniond q_att;
                get_q_from_dcm(q_att, w_R_b);
                imu_msg.orientation.w = q_att.w();
                imu_msg.orientation.x = q_att.x();
                imu_msg.orientation.y = q_att.y();
                imu_msg.orientation.z = q_att.z();

                imu_msg.angular_velocity.x = 0.0;
                imu_msg.angular_velocity.y = 0.0;
                imu_msg.angular_velocity.z = 0.0;

                Eigen::Vector3d body_imu_acc = w_R_b.transpose() * (s_.acc + Eigen::Vector3d(0, 0, CONSTANTS_ONE_G));
                imu_msg.linear_acceleration.x = body_imu_acc(0);
                imu_msg.linear_acceleration.y = body_imu_acc(1);
                imu_msg.linear_acceleration.z = body_imu_acc(2);

                return imu_msg;
            }

            nav_msgs::Odometry getOdomfromState() {
                nav_msgs::Odometry res;
                res.header.stamp = ros::Time::now();
                res.header.frame_id = "world";
                /* get odom data */
                res.pose.pose.position.x = s_.pos(0);
                res.pose.pose.position.y = s_.pos(1);
                res.pose.pose.position.z = s_.pos(2);

                Eigen::Matrix3d w_R_b;
                get_dcm_from_euler(w_R_b, Eigen::Vector3d(s_.roll, s_.pitch, s_.yaw));
                Eigen::Quaterniond q_att;
                get_q_from_dcm(q_att, w_R_b);
                res.pose.pose.orientation.w = q_att.w();
                res.pose.pose.orientation.x = q_att.x();
                res.pose.pose.orientation.y = q_att.y();
                res.pose.pose.orientation.z = q_att.z();

                res.twist.twist.linear.x = s_.vel(0);
                res.twist.twist.linear.y = s_.vel(1);
                res.twist.twist.linear.z = s_.vel(2);
                return res;
            };

        private:
            inline double normalize_theta(double theta) {
                if (theta >= -M_PI && theta < M_PI)
                    return theta;
                
                double multiplier = std::floor(theta / (2*M_PI));
                theta = theta - multiplier*2*M_PI;
                if (theta >= M_PI)
                    theta -= 2*M_PI;
                if (theta < -M_PI)
                    theta += 2*M_PI;

                return theta;
            }
            inline double constraint(double x, double a, double b) {
                return std::max(std::min(x, b), a);
            }

            inline void get_dcm_from_q(Eigen::Matrix3d &dcm, const Eigen::Quaterniond &q) {
                double a = q.w();
                double b = q.x();
                double c = q.y();
                double d = q.z();
                double aSq = a*a;
                double bSq = b*b;
                double cSq = c*c;
                double dSq = d*d;
                dcm(0, 0) = aSq + bSq - cSq - dSq; 
                dcm(0, 1) = 2 * (b * c - a * d);
                dcm(0, 2) = 2 * (a * c + b * d);
                dcm(1, 0) = 2 * (b * c + a * d);
                dcm(1, 1) = aSq - bSq + cSq - dSq;
                dcm(1, 2) = 2 * (c * d - a * b);
                dcm(2, 0) = 2 * (b * d - a * c);
                dcm(2, 1) = 2 * (a * b + c * d);
                dcm(2, 2) = aSq - bSq - cSq + dSq;
            }

            inline void get_euler_from_R(Eigen::Vector3d &e, const Eigen::Matrix3d &R) {
                double phi = std::atan2(R(2, 1), R(2, 2));
                double theta = std::asin(-R(2, 0));
                double psi = std::atan2(R(1, 0), R(0, 0));
                double pi = M_PI;

                if (std::abs(theta - pi/2.0) < 1.0e-3) {
                    phi = 0.0;
                    psi = std::atan2(R(1, 2), R(0, 2));
                } else if (std::abs(theta + pi/2.0) < 1.0e-3) {
                    phi = 0.0;
                    psi = std::atan2(-R(1, 2), -R(0, 2));
                }
                e(0) = phi;
                e(1) = theta;
                e(2) = psi;
            }

            inline void get_euler_from_q(Eigen::Vector3d &e, const Eigen::Quaterniond &q) {
                Eigen::Matrix3d temp_R;
                get_dcm_from_q(temp_R, q);
                get_euler_from_R(e, temp_R);
            }

            inline void get_dcm_from_euler(Eigen::Matrix3d &R, const Eigen::Vector3d &e) {
                double a = e(2);
                double b = e(1);
                double c = e(0);
                double ca = std::cos(a);
                double sa = std::sin(a);
                double cb = std::cos(b);
                double sb = std::sin(b);
                double cc = std::cos(c);
                double sc = std::sin(c);
                R(0,0) = ca * cb;
                R(0,1) = ca * sb * sc - sa * cc;
                R(0,2) = ca * sb * cc + sa * sc;
                R(1,0) = sa * cb;
                R(1,1) = sa * sb * sc + ca * cc;
                R(1,2) = sa * sb * cc - ca * sc;
                R(2,0) = - sb;
                R(2,1) = cb * sc;
                R(2,2) = cb * cc;
            }

            inline void get_q_from_dcm(Eigen::Quaterniond &q, const Eigen::Matrix3d &dcm) {
                float t = dcm.trace();
                if ( t > 0.0f ) {
                    t = sqrt(1.0f + t);
                    q.w() = 0.5f * t;
                    t = 0.5f / t;
                    q.x() = (dcm(2,1) - dcm(1,2)) * t;
                    q.y() = (dcm(0,2) - dcm(2,0)) * t;
                    q.z() = (dcm(1,0) - dcm(0,1)) * t;
                } else if (dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2)) {
                    t = sqrt(1.0f + dcm(0,0) - dcm(1,1) - dcm(2,2));
                    q.x() = 0.5f * t;
                    t = 0.5f / t;
                    q.w() = (dcm(2,1) - dcm(1,2)) * t;
                    q.y() = (dcm(1,0) + dcm(0,1)) * t;
                    q.z() = (dcm(0,2) + dcm(2,0)) * t;
                } else if (dcm(1,1) > dcm(2,2)) {
                    t = sqrt(1.0f - dcm(0,0) + dcm(1,1) - dcm(2,2));
                    q.y() = 0.5f * t;
                    t = 0.5f / t;
                    q.w() = (dcm(0,2) - dcm(2,0)) * t;
                    q.x() = (dcm(1,0) + dcm(0,1)) * t;
                    q.z() = (dcm(2,1) + dcm(1,2)) * t;
                } else {
                    t = sqrt(1.0f - dcm(0,0) - dcm(1,1) + dcm(2,2));
                    q.z() = 0.5f * t;
                    t = 0.5f / t;
                    q.w() = (dcm(1,0) - dcm(0,1)) * t;
                    q.x() = (dcm(0,2) + dcm(2,0)) * t;
                    q.y() = (dcm(2,1) + dcm(1,2)) * t;
                }
            }

            State s_;
            Control u_;
            bool ctrl_set_{false};
            bool state_set_{false};
            double max_vz_{5.0};
            double scale_acc_thrust_{0.0};
            AlphaFilter<double> thrust_filter_;
            AlphaRadFilter<double> roll_filter_;
            AlphaRadFilter<double> pitch_filter_;
            AlphaRadFilter<double> yaw_filter_;
            bool on_ground_{true};
    };
} // namespace fast_model
#endif