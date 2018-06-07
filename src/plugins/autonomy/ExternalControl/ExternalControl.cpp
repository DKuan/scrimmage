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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControl.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/math/State.h>
#include <scrimmage/sensor/Sensor.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/string.hpp>

#include <chrono> // NOLINT
#include <thread> // NOLINT

namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ExternalControl, ExternalControl_plugin)

namespace scrimmage {
namespace autonomy {

ExternalControl::ExternalControl() :
    reward_range(-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()) {}

void ExternalControl::init(std::map<std::string, std::string> &params) {
    print_err_on_exit = false;
    return;
}

bool ExternalControl::step_autonomy(double t, double dt) {
    return false;
}

std::pair<bool, double> ExternalControl::calc_reward(double t, double dt) {
    return {false, 0.0};
}

bool ExternalControl::check_action(
        const scrimmage_proto::Action &action,
        uint64_t discrete_action_size,
        uint64_t continuous_action_size) {
    if (action.done()) {
        return false;
    } else if (static_cast<uint64_t>(action.discrete_size()) != discrete_action_size) {
        std::cout << "received discrete external action of length "
            << action.discrete_size() << " (need length "
            << discrete_action_size << ")" << std::endl;
        return false;
    } else if (static_cast<uint64_t>(action.continuous_size()) != continuous_action_size) {
        std::cout << "received continuous external action of length "
            << action.continuous_size() << " (need length "
            << continuous_action_size << ")" << std::endl;
        return false;
    }
    return true;
}

} // namespace autonomy
} // namespace scrimmage
