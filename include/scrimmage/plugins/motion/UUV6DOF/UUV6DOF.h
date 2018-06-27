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
#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UUV6DOF_UUV6DOF_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UUV6DOF_UUV6DOF_H_

#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/CSV.h>

#include <scrimmage/plugins/motion/RigidBody6DOF/RigidBody6DOFBase.h>

#include <map>
#include <string>

namespace scrimmage {
namespace motion {
class UUV6DOF : public scrimmage::motion::RigidBody6DOFBase {
 public:
    enum ModelParams {
        U = 0,
        V,
        W,
        P,
        Q,
        R,
        U_dot,
        V_dot,
        W_dot,
        P_dot,
        Q_dot,
        R_dot,
        Uw,
        Vw,
        Ww,
        Xw,
        Yw,
        Zw,
        q0,
        q1,
        q2,
        q3,
        MODEL_NUM_ITEMS
    };

    UUV6DOF();

    bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) override;

    bool step(double time, double dt) override;

    void model(const vector_t &x , vector_t &dxdt , double t) override;

    class Controller : public scrimmage::Controller {
     public:
        virtual Eigen::Vector2d &u() = 0;
    };

 protected:
    int throttle_idx_ = 0;
    int elevator_idx_ = 0;
    int aileron_idx_ = 0;
    int rudder_idx_ = 0;

    double thrust_ = 0;
    double throttle_ = 0;
    double delta_elevator_ = 0;
    double delta_rudder_ = 0;

    Eigen::Quaterniond rot_180_x_axis_;

    Eigen::Matrix3d I_;
    Eigen::Matrix3d I_inv_;

    scrimmage::Quaternion quat_body_;
    Eigen::Vector3d force_ext_body_;

    // Logging utility
    bool write_csv_ = false;
    CSV csv_;

    double buoyancy_ = 306.0;

    Eigen::Matrix<double, 6, 6> added_mass_;

    double Xuu_ = -1.620;
    double Yvv_ = -131.0;
    double Zww_ = -131.0;
    double Mww_ = +3.180;
    double Yrr_ = +0.632;
    double Mqq_ = -9.400;

    // double rho_ = 1030.0;
    // double c_d_ = 0.27;
    // double A_f_ = 0.0285;


    Eigen::Vector3d c_g_;
    Eigen::Vector3d c_b_;

    double thrust_min_ = -2.0;
    double thrust_max_ = +3.6936;
    double delta_elevator_min_ = -0.2373648;
    double delta_elevator_max_ = +0.2373648;
    double delta_rudder_min_ = -0.2373648;
    double delta_rudder_max_ = +0.2373648;


 private:
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UUV6DOF_UUV6DOF_H_
