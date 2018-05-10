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

#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/ExternalControl.pb.h>

#include <scrimmage/plugins/autonomy/RLSimple/RLSimple.h>

#include <iostream>

namespace sp = scrimmage_proto;
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, RLSimple, RLSimple_plugin)

void RLSimple::init(std::map<std::string, std::string> &params) {
    x_discrete_ = sc::str2bool(params.at("x_discrete"));
    y_discrete_ = sc::str2bool(params.at("y_discrete"));
    ctrl_y_ = sc::str2bool(params.at("ctrl_y"));

    using Type = sc::VariableIO::Type;
    using Dir = sc::VariableIO::Direction;

    output_vel_x_idx_ = vars_.declare(Type::velocity_x, Dir::Out);
    output_vel_y_idx_ = vars_.declare(Type::velocity_y, Dir::Out);
    uint8_t output_vel_z_idx = vars_.declare(Type::velocity_z, Dir::Out);

    vars_.output(output_vel_x_idx_, 0);
    vars_.output(output_vel_y_idx_, 0);
    vars_.output(output_vel_z_idx, 0);

    radius_ = std::stod(params.at("radius"));
    ExternalControl::init(params);
}

std::pair<bool, double> RLSimple::calc_reward(double t, double dt) {
    const bool done = false;
    const bool reward = state_->pos()(0) < radius_;
    return {done, reward};
}

double RLSimple::action_getter(bool discrete, int idx) {
    if (discrete) {
        return action_.discrete(idx) ? 1 : -1;
    } else {
        return action_.continuous(idx);
    }
}

bool RLSimple::step_autonomy(double t, double dt) {
    if (action_.done()) return false;

    int num_discrete = x_discrete_ + (ctrl_y_ && y_discrete_);
    int num_continuous = !x_discrete_ + (ctrl_y_ && !y_discrete_);

    if (!check_action(action_, num_discrete, num_continuous)) return false;

    double x_vel = action_getter(x_discrete_, 0);
    double y_vel = ctrl_y_ ? action_getter(y_discrete_, 1) : 0;

    vars_.output(output_vel_x_idx_, x_vel);
    vars_.output(output_vel_y_idx_, y_vel);
    return true;
}

scrimmage_proto::SpaceParams RLSimple::action_space_params() {
    sp::SpaceParams space_params;

    // y control
    sp::SingleSpaceParams *x_ctrl_params = space_params.add_params();
    x_ctrl_params->set_num_dims(1);
    x_ctrl_params->set_discrete(x_discrete_);
    x_ctrl_params->add_minimum(x_discrete_ ? 0 : -std::numeric_limits<double>::infinity());
    x_ctrl_params->add_maximum(x_discrete_ ? 1 : std::numeric_limits<double>::infinity());

    // y control
    if (ctrl_y_) {
        sp::SingleSpaceParams *y_ctrl_params = space_params.add_params();
        y_ctrl_params->set_num_dims(1);
        y_ctrl_params->set_discrete(y_discrete_);
        y_ctrl_params->add_minimum(y_discrete_ ? 0 : -std::numeric_limits<double>::infinity());
        y_ctrl_params->add_maximum(y_discrete_ ? 1 : std::numeric_limits<double>::infinity());
    }

    return space_params;
}
