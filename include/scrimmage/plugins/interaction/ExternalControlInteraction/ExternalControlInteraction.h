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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_EXTERNALCONTROLINTERACTION_EXTERNALCONTROLINTERACTION_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_EXTERNALCONTROLINTERACTION_EXTERNALCONTROLINTERACTION_H_

#include <scrimmage/simcontrol/EntityInteraction.h>

#include <map>
#include <list>
#include <string>
#include <vector>
#include <utility>

namespace scrimmage {
class Entity;
using EntityPtr = std::shared_ptr<Entity>;

namespace autonomy {
class ExternalControl;
using ExternalControlPtr = std::shared_ptr<ExternalControl>;

class ExternalControlClient;
using ExternalControlClientPtr = std::shared_ptr<ExternalControlClient>;
}

namespace interaction {

class ExternalControlInteraction : public scrimmage::EntityInteraction {
 public:
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                 double t, double dt) override;
    void close(double t) override;

 protected:
    bool send_env();

    scrimmage_proto::ActionResults get_action_result(double t, double dt);

    bool env_sent_ = false;

    std::vector<autonomy::ExternalControlPtr> ext_ctrl_vec_;
    autonomy::ExternalControlClientPtr external_control_client_;
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_EXTERNALCONTROLINTERACTION_EXTERNALCONTROLINTERACTION_H_
