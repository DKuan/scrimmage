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
#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControlClient.h>

#include <grpc++/grpc++.h>

#include <iostream>

#include <boost/optional.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

namespace scrimmage {
namespace autonomy {

// cppcheck-suppress passedByValue
ExternalControlClient::ExternalControlClient(std::shared_ptr<grpc::Channel> channel) :
    stub_(scrimmage_proto::ExternalControl::NewStub(channel)) {}

bool ExternalControlClient::send_environments(scrimmage_proto::Environments &envs) {
    sp::Empty reply;
    grpc::ClientContext context;
    grpc::Status status = stub_->SendEnvironments(&context, envs, &reply);
    return status.ok();
}

boost::optional<scrimmage_proto::Actions>
ExternalControlClient::send_action_results(
        scrimmage_proto::ActionResults &action_results) {
    // Connection timeout in seconds
    unsigned int client_connection_timeout = 5;

    grpc::ClientContext context;

    // https://github.com/grpc/grpc/issues/3954
    std::chrono::system_clock::time_point deadline =
        std::chrono::system_clock::now() + std::chrono::seconds(client_connection_timeout);

    context.set_deadline(deadline);

    sp::Actions actions;
    grpc::Status status = stub_->SendActionResults(&context, action_results, &actions);
    if (!status.ok()) {
        std::cout << "ExternalControlClient received bad grpc status" << std::endl;
        return boost::none;
    } else {
        return boost::optional<sp::Actions>(actions);
    }
}
} // namespace autonomy
} // namespace scrimmage
