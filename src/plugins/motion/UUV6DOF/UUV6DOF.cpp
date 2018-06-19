/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>

#include <scrimmage/plugins/motion/UUV6DOF/UUV6DOF.h>

#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::MotionModel,
                scrimmage::motion::UUV6DOF,
                UUV6DOF_plugin)

namespace scrimmage {
namespace motion {

UUV6DOF::UUV6DOF() : length_(1.0),
                    enable_gravity_(false) {
    Eigen::AngleAxisd aa(M_PI, Eigen::Vector3d::UnitX());
    rot_180_x_axis_ = Eigen::Quaterniond(aa);
    x_.resize(MODEL_NUM_ITEMS);
}

bool UUV6DOF::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
    Eigen::Vector3d &pos = state_->pos();

    // Need to rotate axes by 180 degrees around X-axis SCRIMMAGE's global
    // frame uses Z-axis pointing up. Many aircraft equations of motion are
    // specified with Z-axis pointing down.
    quat_body_ = rot_180_x_axis_ * state_->quat();
    quat_body_.set(sc::Angles::angle_pi(quat_body_.roll()+M_PI),
                   quat_body_.pitch(), quat_body_.yaw());

    x_[U] = state_->vel()(0);
    x_[V] = 0;
    x_[W] = 0;

    x_[P] = 0;
    x_[Q] = 0;
    x_[R] = 0;

    x_[Uw] = state_->vel()(0);
    x_[Vw] = 0;
    x_[Ww] = 0;

    x_[Xw] = pos(0);
    x_[Yw] = pos(1);
    x_[Zw] = pos(2);

    x_[q0] = quat_body_.w();
    x_[q1] = quat_body_.x();
    x_[q2] = quat_body_.y();
    x_[q3] = quat_body_.z();

    // Parse XML parameters
    g_ = sc::get<double>("gravity_magnitude", params, 9.81);
    mass_ = sc::get<double>("mass", params, 1.2);
    buoyancy_ = sc::get<double>("buoyancy", params, buoyancy_);

    // Get the inertia matrix
    std::vector<std::vector<std::string>> vecs;
    std::string inertia_matrix = sc::get<std::string>("inertia_matrix",
                                                      params, "");

    // Parse inertia matrix
    bool valid_inertia = false;
    if (!sc::get_vec_of_vecs(inertia_matrix, vecs)) {
        cout << "Failed to parse inertia_matrix:" << inertia_matrix << endl;
    } else {
        int row = 0;
        for (std::vector<std::string> vec : vecs) {
            if (vec.size() != 3) {
                cout << "Invalid vector size in: " << inertia_matrix << endl;
                break;
            }
            for (int i = 0; i < 3; i++) {
                I_(row, i) = std::stod(vec[i]);
            }
            row++;
        }
        if (row == 3) {
            valid_inertia = true;
        }
    }
    if (!valid_inertia) {
        cout << "Using identity matrix for inertia." << endl;
        I_ = Eigen::Matrix3d::Identity();
    }
    I_inv_ = I_.inverse();

    // Should we write a CSV file? What values should be written?
    write_csv_ = sc::get<bool>("write_csv", params, false);
    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv");
        cout << "Writing log to " + parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv" << endl;

        csv_.set_column_headers(sc::CSV::Headers{"t",
                    "x", "y", "z",
                    "U", "V", "W",
                    "P", "Q", "R",
                    "U_dot", "V_dot", "W_dot",
                    "P_dot", "Q_dot", "R_dot",
                    "roll", "pitch", "yaw",
                    });
    }

    // rho_ = sc::get<double>("rho", params, rho_);
    // c_d_ = sc::get<double>("c_d", params, c_d_);
    // A_f_ = sc::get<double>("A_f", params, A_f_);

    Xuu_ = sc::get<double>("Xuu", params, Xuu_);
    Yvv_ = sc::get<double>("Yvv", params, Yvv_);
    Zww_ = sc::get<double>("Zww", params, Zww_);
    Mww_ = sc::get<double>("Mww", params, Mww_);
    Yrr_ = sc::get<double>("Yrr", params, Yrr_);
    Mqq_ = sc::get<double>("Mqq", params, Mqq_);

    {
        // Parse center of gravity
        std::vector<double> c_g_vec;
        if (sc::get_vec<double>("c_g", params, ", ", c_g_vec, 3)) {
            c_g_ = sc::vec2eigen(c_g_vec);
        } else {
            cout << "Warning: Invalid center of gravity, c_g." << endl;
        }
    }

    {
        // Parse center of buoyancy
        std::vector<double> c_b_vec;
        if (sc::get_vec<double>("c_b", params, ", ", c_b_vec, 3)) {
            c_b_ = sc::vec2eigen(c_b_vec);
        } else {
            cout << "Warning: Invalid center of buoyancy, c_b." << endl;
        }
    }

    return true;
}

bool UUV6DOF::step(double time, double dt) {
    // Saturate inputs

    // Cache values to calculate changes:
    Eigen::Vector3d prev_linear_vel_ENU(x_[Uw], x_[Vw], x_[Ww]);
    Eigen::Vector3d prev_angular_vel(x_[P], x_[Q], x_[R]);

    // Apply any external forces (todo)
    force_ext_body_ = quat_body_.rotate_reverse(ext_force_);
    ext_force_ = Eigen::Vector3d::Zero(); // reset ext_force_ member variable

    ode_step(dt);

    quat_body_.set(x_[q0], x_[q1], x_[q2], x_[q3]);
    quat_body_.normalize();

    // Calculate change in velocity to populate acceleration elements
    Eigen::Vector3d linear_vel_ENU(x_[Uw], x_[Vw], x_[Ww]);
    Eigen::Vector3d linear_acc_ENU = (linear_vel_ENU - prev_linear_vel_ENU) / dt;
    Eigen::Vector3d angular_vel(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d angular_acc = (angular_vel - prev_angular_vel) / dt;
    Eigen::Vector3d angular_acc_FLU(angular_acc(0), -angular_acc(1), -angular_acc(2));

    // Rotate back to Z-axis pointing up
    state_->quat() = rot_180_x_axis_ * quat_body_;
    state_->quat().set(sc::Angles::angle_pi(state_->quat().roll()+M_PI),
                       state_->quat().pitch(), state_->quat().yaw());


    Eigen::Vector3d angvel_b_e_bodyRef = quat_body_.rotate(angular_vel);
    Eigen::Vector3d angvel_b_e_ENU;
    angvel_b_e_ENU << angvel_b_e_bodyRef[0], -angvel_b_e_bodyRef[1], -angvel_b_e_bodyRef[2];
    state_->set_ang_vel(angvel_b_e_ENU);


    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << x_[Uw], x_[Vw], x_[Ww];

    linear_accel_body_ = state_->quat().rotate_reverse(linear_acc_ENU);
    ang_accel_body_ = angular_acc_FLU;

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", time},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"U_dot", x_[U_dot]},
                {"V_dot", x_[V_dot]},
                {"W_dot", x_[W_dot]},
                {"P_dot", x_[P_dot]},
                {"Q_dot", x_[Q_dot]},
                {"R_dot", x_[R_dot]},
                {"roll", quat_body_.roll()},
                {"pitch", quat_body_.pitch()},
                {"yaw", quat_body_.yaw()}});
    }

    return true;
}

void UUV6DOF::model(const vector_t &x, vector_t &dxdt, double t) {
    // Calculate velocity magnitude (handle zero velocity)
    double V_tau = sqrt(pow(x_[U], 2) + pow(x_[V], 2) + pow(x_[W], 2));
    if (std::abs(V_tau) < std::numeric_limits<double>::epsilon()) {
        V_tau = 0.00001;
    }

    // Calculate force from weight in body frame:
    Eigen::Vector3d gravity_vector(0, 0, +mass_*g_);
    Eigen::Vector3d F_weight = quat_body_.rotate_reverse(gravity_vector);

    Eigen::Vector3d buoyancy_vector (0, 0, buoyancy_);
    Eigen::Vector3d F_buoyancy = quat_body_.rotate_reverse(buoyancy_vector);

    Eigen::Vector3d F_hydro = F_weight - F_buoyancy;
    Eigen::Vector3d Moments_hydro = c_g_.cross(F_weight) - c_b_.cross(F_buoyancy);

    Eigen::Vector3d F_drag(Xuu_ * x[U] * x[U],
                           Yvv_ * x[V] * x[V],
                           Zww_ * x[W] * x[W]);


    Eigen::Vector3d F_thrust(0, 0, 0);
    Eigen::Vector3d F_total = F_hydro + F_drag;

    // Calculate body frame linear velocities
    dxdt[U] = x[V]*x[R] - x[W]*x[Q] + F_total(0) / mass_;
    dxdt[V] = x[W]*x[P] - x[U]*x[R] + F_total(1) / mass_;
    dxdt[W] = x[U]*x[Q] - x[V]*x[P] + F_total(2) / mass_;

    // Calculate moments;
    Eigen::Vector3d Moments_thrust(0, 0, 0); // no moment from thrust
    Eigen::Vector3d Moments_torque(0, 0, 0); // no moment from torque

    // Sum moments
    Eigen::Vector3d Moments_total = Moments_hydro + Moments_thrust + Moments_torque;

    // Calculate rotational velocites
    Eigen::Vector3d pqr(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d pqr_dot = I_inv_ * (Moments_total - pqr.cross(I_*pqr));
    dxdt[P] = pqr_dot(0);
    dxdt[Q] = pqr_dot(1);
    dxdt[R] = pqr_dot(2);

    // Compute quaternion derivatives
    double lambda = 1 - (pow(x[q0], 2) + pow(x[q1], 2) + pow(x[q2], 2) + pow(x[q3], 2));
    dxdt[q0] = -0.5 * (x[q1]*x[P] + x[q2]*x[Q] + x[q3]*x[R]) + lambda * x[q0];
    dxdt[q1] = +0.5 * (x[q0]*x[P] + x[q2]*x[R] - x[q3]*x[Q]) + lambda * x[q1];
    dxdt[q2] = +0.5 * (x[q0]*x[Q] + x[q3]*x[P] - x[q1]*x[R]) + lambda * x[q2];
    dxdt[q3] = +0.5 * (x[q0]*x[R] + x[q1]*x[Q] - x[q2]*x[P]) + lambda * x[q3];

    // Normalize quaternion
    sc::Quaternion quat(x[q0], x[q1], x[q2], x[q3]);
    quat.w() = x[q0];
    quat.x() = x[q1];
    quat.y() = x[q2];
    quat.z() = x[q3];
    quat.normalize();

    // Integrate local velocities to compute local positions
    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = quat.rotate(vel_local); // rot * vel_local;
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = -vel_world(1); // Due to rotated frame
    dxdt[Zw] = -vel_world(2); // Due to rotated frame

    // Integrate local accelerations to compute global velocities
    Eigen::Vector3d acc_local = F_total / mass_;
    Eigen::Vector3d acc_world = quat.rotate(acc_local); // rot * acc_local;
    dxdt[Uw] = acc_world(0);
    dxdt[Vw] = -acc_world(1); // Due to rotated frame
    dxdt[Ww] = -acc_world(2); // Due to rotated frame

    // Accelerations get updated based on change in velocities
    dxdt[U_dot] = 0;
    dxdt[V_dot] = 0;
    dxdt[W_dot] = 0;
    dxdt[P_dot] = 0;
    dxdt[Q_dot] = 0;
    dxdt[R_dot] = 0;
}
} // namespace motion
} // namespace scrimmage
