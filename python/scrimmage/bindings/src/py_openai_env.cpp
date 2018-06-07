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

#include "py_openai_env.h"

#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <pybind11/pybind11.h>

#include <string>
#include <algorithm>
#include <limits>

#include <boost/optional.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/min_element.hpp>

namespace py = pybind11;
namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;

namespace {
// https://stackoverflow.com/a/48164204
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
} // namespace

ScrimmageOpenAIEnv::ScrimmageOpenAIEnv(
        const std::string &mission_file,
        bool enable_gui,
        bool combine_actors,
        bool global_sensor) :
    mission_file_(mission_file),
    enable_gui_(enable_gui),
    combine_actors_(combine_actors),
    global_sensor(global_sensor) {

    py::module warnings_module = py::module::import("warnings");
    warning_function_ = warnings_module.attr("warn");

    auto mp = std::make_shared<sc::MissionParse>();
    if (!mp->parse(mission_file)) {
        std::cout << "Failed to parse file: " << mission_file << std::endl;
    }

    // Handle kill signals
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    shutdown_handler = [&](int /*s*/){
        std::cout << std::endl << "Exiting gracefully" << std::endl;
        simcontrol_.force_exit();
    };
    sa.sa_handler = signal_handler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    log_ = sc::preprocess_scrimmage(mp, simcontrol_);
    if (log_ == nullptr) {
        py::print("scrimmage initialization unsuccessful");
    }

    if (!simcontrol_.generate_entities(0)) {
        py::print("scrimmage entity generation unsuccessful");
    }

    for (auto &e : simcontrol_.ents()) {
        for (auto &a : e->autonomies()) {
            auto a_cast = std::dynamic_pointer_cast<sc::autonomy::ExternalControl>(a);
            if (a_cast) {
                ext_ctrl_vec_.push_back(a_cast);

                std::vector<ScrimmageOpenAISensor> vec;
                for (auto &s : a_cast->parent()->sensors()) {
                    auto s_cast =
                      std::dynamic_pointer_cast<sc::sensor::ScrimmageOpenAISensor>(s.second);
                    if (s_cast) {
                        vec.push_back(s_cast);
                    }
                }
                ext_sensor_vec_.push_back(vec);
            }
        }
    }

    reward_range = pybind11::tuple(2);
    set_reward_range();
    create_action_space();
    create_observation_space();
}

void ScrimmageOpenAIEnv::to_continuous(std::vector<std::pair<double, double>> &p,
                                       pybind11::list &minima,
                                       pybind11::list &maxima) {

    for (auto &value : p) {
        py::list min_max;
        minima.append(value.first);
        maxima.append(value.second);
    }
}

void ScrimmageOpenAIEnv::to_discrete(std::vector<double> &p, py::list &maxima) {
    for (auto &value : p) {
        maxima.append(value);
    }
}

void ScrimmageOpenAIEnv::create_action_space() {

    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list discrete_maxima;
        py::list continuous_minima;
        py::list continuous_maxima;

        for (auto &autonomy : ext_ctrl_vec_) {
            to_discrete(autonomy->action_space.discrete_maxima, discrete_maxima);
            to_continuous(autonomy->action_space.continuous_extrema, continuous_minima, continuous_maxima);
        }

        action_space =
            create_space(discrete_maxima, continuous_minima, continuous_maxima);

    } else {

        py::list action_spaces;

        for (auto &autonomy : ext_ctrl_vec_) {
            py::list discrete_maxima;
            py::list continuous_minima;
            py::list continuous_maxima;

            to_discrete(autonomy->action_space.discrete_maxima, discrete_maxima);
            to_continuous(autonomy->action_space.continuous_extrema, continuous_minima, continuous_maxima);

            auto space =
                create_space(discrete_maxima, continuous_minima, continuous_maxima);
            action_spaces.append(space);
        }

        py::module gym = py::module::import("gym");
        py::object spaces = gym.attr("spaces");
        py::object tuple_space = spaces.attr("Tuple");
        action_space = tuple_space(action_spaces);
    }
}

void ScrimmageOpenAIEnv::create_observation_space() {
    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list discrete_maxima;
        py::list continuous_minima;
        py::list continuous_maxima;

        for (auto &v : ext_sensor_vec_) {
            for (auto &s : v) {
                to_discrete(s->observation_space.discrete_maxima, discrete_maxima);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);
            }
        }

        observation_space =
            create_space(discrete_maxima, continuous_minima, continuous_maxima);

    } else {

        py::list observation_spaces;

        for (auto &v : ext_sensor_vec_) {
            for (auto &s : v) {
                py::list discrete_maxima;
                py::list continuous_minima;
                py::list continuous_maxima;

                to_discrete(s->observation_space.discrete_maxima, discrete_maxima);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);

                auto space =
                    create_space(discrete_maxima, continuous_minima, continuous_maxima);
                observation_spaces.append(space);
            }
        }

        py::module gym = py::module::import("gym");
        py::object spaces = gym.attr("spaces");
        py::object tuple_space = spaces.attr("Tuple");
        observation_space = tuple_space(observation_spaces);
    }
}

void ScrimmageOpenAIEnv::set_reward_range() {
    if (ext_ctrl_vec_.size() == 1) {
        reward_range[0] = ext_ctrl_vec_[0]->reward_range.first;
        reward_range[1] = ext_ctrl_vec_[0]->reward_range.second;
    } else {

        auto get_min_reward = [&](auto &a) {return a->reward_range.first;};
        auto get_max_reward = [&](auto &a) {return a->reward_range.second;};
        auto min_reward = ext_ctrl_vec_ | ba::transformed(get_min_reward);
        auto max_reward = ext_ctrl_vec_ | ba::transformed(get_max_reward);

        if (combine_actors_) {
            reward_range[0] = boost::accumulate(min_reward, 0.0);
            reward_range[1] = boost::accumulate(max_reward, 0.0);
        } else {
            double mx = -std::numeric_limits<double>::infinity();
            double mn = std::numeric_limits<double>::infinity();

            for (auto r : min_reward) mn = std::min(mn, r);
            for (auto r : max_reward) mx = std::max(mx, r);

            reward_range[0] = mn;
            reward_range[1] = mx;
        }
    }
}

pybind11::object ScrimmageOpenAIEnv::create_space(
        pybind11::list discrete_maxima,
        pybind11::list continuous_minima,
        pybind11::list continuous_maxima) {

    py::module np = py::module::import("numpy");
    py::object np_array = np.attr("array");
    py::object np_float32 = np.attr("float32");

    py::module gym = py::module::import("gym");
    py::object spaces = gym.attr("spaces");

    py::object gym_discrete_space = spaces.attr("Discrete");
    py::object gym_multidiscrete_space = spaces.attr("MultiDiscrete");
    py::object gym_box_space = spaces.attr("Box");
    py::object gym_tuple_space = spaces.attr("Tuple");

    py::object discrete_space = py::len(discrete_maxima) == 1 ?
        gym_discrete_space(discrete_maxima[0]) :
        gym_multidiscrete_space(discrete_maxima);

    py::object continuous_space = gym_box_space(
        np_array(continuous_minima),
        np_array(continuous_maxima),
        py::none(),
        np_float32);

    int len_discrete = py::len(discrete_maxima);
    int len_continuous = py::len(continuous_minima);
    if (len_discrete != 0 && len_continuous != 0) {
        return gym_tuple_space(discrete_space, continuous_space);
    } else if (len_discrete != 0) {
        return discrete_space;
    } else if (len_continuous != 0) {
        return continuous_space;
    } else {
        // TODO: error handling
        return pybind11::object();
    }
}

void ScrimmageOpenAIEnv::run_simcontrol() {
    py::print("hello world");
    py::print("hello1");
    simcontrol_.pause(false);
    py::print("hello2");
    simcontrol_.run();
    py::print("hello3");
}

void ScrimmageOpenAIEnv::render() {
    warning_function_("render must be set in gym.make with enable_gui kwargs");
}

pybind11::tuple ScrimmageOpenAIEnv::step(pybind11::object action) {}

pybind11::object ScrimmageOpenAIEnv::reset() {}

void ScrimmageOpenAIEnv::close() {}

void ScrimmageOpenAIEnv::seed(pybind11::object _seed) {}

void add_openai_env(pybind11::module &m) {
    py::class_<ScrimmageOpenAIEnv>(m, "ScrimmageOpenAIEnv")
        .def(py::init<std::string&, bool, bool, bool>(),
            "docs",
            py::arg("mission_file"),
            py::arg("enable_gui") = false,
            py::arg("combine_actors") = false,
            py::arg("global_sensor") = false)
        .def("step", &ScrimmageOpenAIEnv::step)
        .def_readwrite("reward_range", &ScrimmageOpenAIEnv::reward_range)
        .def_readwrite("action_space", &ScrimmageOpenAIEnv::action_space)
        .def_readwrite("observation_space", &ScrimmageOpenAIEnv::observation_space)
        .def("reset", &ScrimmageOpenAIEnv::reset)
        .def("close", &ScrimmageOpenAIEnv::close)
        .def("seed", &ScrimmageOpenAIEnv::seed)
        .def("render", &ScrimmageOpenAIEnv::render);
}
