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

#include <scrimmage/plugins/sensor/RLSimpleSensor/RLSimpleSensor.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/ExternalControl.pb.h>

#include <boost/range/adaptor/map.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace ba = boost::adaptors;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::RLSimpleSensor, RLSimpleSensor_plugin)

namespace scrimmage {
namespace sensor {

void RLSimpleSensor::init(std::map<std::string, std::string> &/*params*/) {
    const double inf = std::numeric_limits<double>::infinity();
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
}

const EnvValues &RLSimpleSensor::get_observations() {
    observations.continuous[0] = parent_->state()->pos()(0);
    return observations;
}

} // namespace sensor
} // namespace scrimmage
