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
#include <scrimmage/viewer/Viewer.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <string>
#include <algorithm>
#include <limits>
#include <exception>

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
    spec(py::none()),
    metadata(py::dict()),
    mission_file_(mission_file),
    combine_actors_(combine_actors),
    global_sensor_(global_sensor),
    enable_gui_(enable_gui) {

    py::module warnings_module = py::module::import("warnings");
    warning_function_ = warnings_module.attr("warn");

    mp_ = std::make_shared<sc::MissionParse>();
    reset_scrimmage();

    reward_range = pybind11::tuple(2);
    set_reward_range();
    create_action_space();
    create_observation_space();

    tuple_space = get_gym_space("Tuple").ptr();
    discrete_space = get_gym_space("Discrete").ptr();
    multidiscrete_space = get_gym_space("MultiDiscrete").ptr();
    box_space = get_gym_space("Box").ptr();
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
        py::list discrete_count;
        py::list continuous_minima;
        py::list continuous_maxima;

        for (auto &autonomy : ext_ctrl_vec_) {
            to_discrete(autonomy->action_space.discrete_count, discrete_count);
            to_continuous(autonomy->action_space.continuous_extrema, continuous_minima, continuous_maxima);
        }

        action_space =
            create_space(discrete_count, continuous_minima, continuous_maxima);

    } else {

        py::list action_spaces;

        for (auto &autonomy : ext_ctrl_vec_) {
            py::list discrete_count;
            py::list continuous_minima;
            py::list continuous_maxima;

            to_discrete(autonomy->action_space.discrete_count, discrete_count);
            to_continuous(autonomy->action_space.continuous_extrema, continuous_minima, continuous_maxima);

            auto space =
                create_space(discrete_count, continuous_minima, continuous_maxima);
            action_spaces.append(space);
        }

        py::object tuple_space = get_gym_space("Tuple");
        action_space = tuple_space(action_spaces);
    }
}

void ScrimmageOpenAIEnv::create_observation_space() {

    py::object tuple_space = get_gym_space("Tuple");

    auto create_obs = [&](py::list &discrete_count, py::list &continuous_maxima) -> py::object {

        int len_discrete = py::len(discrete_count);
        int len_continuous = py::len(continuous_maxima);

        py::array_t<int> discrete_array(len_discrete);
        py::array_t<double> continuous_array(len_continuous);

        if (len_discrete > 0 && len_continuous > 0) {
            py::list obs;
            obs.append(discrete_array);
            obs.append(continuous_array);
            return obs;
        } else if (len_continuous > 0) {
            return continuous_array;
        } else {
            return discrete_array;
        }
    };

    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list discrete_count;
        py::list continuous_minima;
        py::list continuous_maxima;

        for (auto &v : ext_sensor_vec_) {
            for (auto &s : v) {
                to_discrete(s->observation_space.discrete_count, discrete_count);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);
            }
        }

        observation_space =
            create_space(discrete_count, continuous_minima, continuous_maxima);

        observation = create_obs(discrete_count, continuous_maxima);

    } else {

        py::list observation_spaces;
        py::list obs;

        for (auto &v : ext_sensor_vec_) {
            for (auto &s : v) {
                py::list discrete_count;
                py::list continuous_minima;
                py::list continuous_maxima;

                to_discrete(s->observation_space.discrete_count, discrete_count);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);

                auto space =
                    create_space(discrete_count, continuous_minima, continuous_maxima);
                observation_spaces.append(space);
                obs.append(create_obs(discrete_count, continuous_maxima));
            }
        }

        observation_space = tuple_space(observation_spaces);
        observation = obs;
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
        pybind11::list discrete_count,
        pybind11::list continuous_minima,
        pybind11::list continuous_maxima) {

    py::module np = py::module::import("numpy");
    py::object np_array = np.attr("array");
    py::object np_float32 = np.attr("float32");

    py::object gym_discrete_space = get_gym_space("Discrete");
    py::object gym_multidiscrete_space = get_gym_space("MultiDiscrete");
    py::object gym_box_space = get_gym_space("Box");
    py::object gym_tuple_space = get_gym_space("Tuple");

    py::object discrete_space = py::len(discrete_count) == 1 ?
        gym_discrete_space(discrete_count[0]) :
        gym_multidiscrete_space(discrete_count);

    py::object continuous_space = gym_box_space(
        np_array(continuous_minima),
        np_array(continuous_maxima),
        py::none(),
        np_float32);

    int len_discrete = py::len(discrete_count);
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

void ScrimmageOpenAIEnv::render() {
    warning_function_("render must be set in gym.make with enable_gui kwargs");
}

void ScrimmageOpenAIEnv::update_observation() {

    auto call_get_obs = [&](auto *data, int &beg_idx, auto sensor, int obs_size) {
        int end_idx = beg_idx + obs_size;
        if (end_idx != beg_idx) {
            sensor->get_observation(data, beg_idx, end_idx);
            beg_idx = end_idx;
        }
    };

    auto init_arrays = [&](py::object obs_space, py::object obs) {
        py::array_t<int> disc_obs;
        py::array_t<double> cont_obs;
        if (PyObject_IsInstance(obs_space.ptr(), tuple_space)) {
            py::list obs_list = obs.cast<py::list>();
            disc_obs = obs_list[0].cast<py::array_t<int>>();
            cont_obs = obs_list[1].cast<py::array_t<double>>();
        } else if (PyObject_IsInstance(obs_space.ptr(), box_space)) {
            cont_obs = obs.cast<py::array_t<double>>();
        } else {
            disc_obs = obs.cast<py::array_t<int>>();
        }

        return std::make_tuple(disc_obs, cont_obs);
    };

    py::array_t<int> disc_obs;
    py::array_t<double> cont_obs;
    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {

        std::tie(disc_obs, cont_obs) = init_arrays(observation_space, observation);
        int disc_beg_idx = 0;
        int cont_beg_idx = 0;
        int* r_disc = static_cast<int *>(disc_obs.request().ptr);
        double* r_cont = static_cast<double *>(cont_obs.request().ptr);

        for (auto &v : ext_sensor_vec_) {
            for (auto &s : v) {
                auto obs_space = s->observation_space;
                call_get_obs(r_disc, disc_beg_idx, s, obs_space.discrete_count.size());
                call_get_obs(r_cont, cont_beg_idx, s, obs_space.continuous_extrema.size());
            }
        }

    } else {
        py::list observation_space_list =
            observation_space.attr("spaces").cast<py::list>();
        py::list observation_list = observation.cast<py::list>();

        for (size_t i = 0; i < ext_sensor_vec_.size(); i++) {
            py::object indiv_space = observation_space_list[i];
            py::object indiv_obs = observation_list[i];
            std::tie(disc_obs, cont_obs) = init_arrays(indiv_space, indiv_obs);
            int disc_beg_idx = 0;
            int cont_beg_idx = 0;
            int* r_disc = static_cast<int *>(disc_obs.request().ptr);
            double* r_cont = static_cast<double *>(cont_obs.request().ptr);

            for (auto &s : ext_sensor_vec_[i]) {
                auto obs_space = s->observation_space;
                call_get_obs(r_disc, disc_beg_idx, s, obs_space.discrete_count.size());
                call_get_obs(r_cont, cont_beg_idx, s, obs_space.continuous_extrema.size());
            }
        }
    }
}

py::object ScrimmageOpenAIEnv::get_gym_space(const std::string &type) {
    return py::module::import("gym").attr("spaces").attr(type.c_str());
}

bool ScrimmageOpenAIEnv::is_gym_instance(pybind11::object &obj, const std::string &type) {
    py::object cls = get_gym_space(type);
    return PyObject_IsInstance(obj.ptr(), cls.ptr());
}

pybind11::object ScrimmageOpenAIEnv::reset() {

    // Handle kill signals
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    shutdown_handler = [&](int /*s*/){
        std::cout << std::endl << "Exiting gracefully" << std::endl;
        simcontrol_.force_exit();
        throw std::exception();
    };
    sa.sa_handler = signal_handler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    reset_scrimmage();
    if (enable_gui_) {
        viewer_thread_ = std::thread(&ScrimmageOpenAIEnv::run_viewer, this);
    }

    update_observation();
    return observation;
}

void ScrimmageOpenAIEnv::reset_scrimmage() {
    if (!mp_->parse(mission_file_)) {
        std::cout << "Failed to parse file: " << mission_file_ << std::endl;
    }
    if (!enable_gui_) {
        mp_->set_time_warp(0);
    }
    log_ = sc::preprocess_scrimmage(mp_, simcontrol_);
    if (!enable_gui_) {
        simcontrol_.pause(false);
    }
    if (log_ == nullptr) {
        py::print("scrimmage initialization unsuccessful");
    }
    loop_number_ = 0;
    if (!simcontrol_.generate_entities(0)) {
        py::print("scrimmage entity generation unsuccessful");
    }

    ext_ctrl_vec_.clear();
    ext_sensor_vec_.clear();
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

}

void ScrimmageOpenAIEnv::run_viewer() {
    py::print("running viewer");
    scrimmage::Viewer viewer;

    auto outgoing = simcontrol_.outgoing_interface();
    auto incoming = simcontrol_.incoming_interface();

    viewer.set_incoming_interface(outgoing);
    viewer.set_outgoing_interface(incoming);
    viewer.set_enable_network(false);
    viewer.init(mp_->attributes()["camera"], mp_->log_dir(), mp_->dt());
    viewer.run();
}

void ScrimmageOpenAIEnv::close() {
    postprocess_scrimmage(mp_, simcontrol_, log_);
}

void ScrimmageOpenAIEnv::seed(pybind11::object _seed) {
    mp_->params()["seed"] = _seed.cast<int>();
}

pybind11::tuple ScrimmageOpenAIEnv::step(pybind11::object action) {

    distribute_action(action);

    simcontrol_.run_single_step(loop_number_++);
    update_observation();

    py::float_ reward;
    py::bool_ done;
    py::dict info;
    std::tie(reward, done, info) = calc_reward();

    return py::make_tuple(observation, reward, done, info);
}

std::tuple<pybind11::float_, pybind11::bool_, pybind11::dict> ScrimmageOpenAIEnv::calc_reward() {

    py::dict info;
    py::list info_done;
    py::list info_reward;

    double t = simcontrol_.t();
    double dt = mp_->dt(); // TODO: rely on time pointer

    double reward = 0;
    bool done = false;

    for (auto &a : ext_ctrl_vec_) {
        double temp_reward;
        bool temp_done;
        std::tie(temp_done, temp_reward) = a->calc_reward(t, dt);

        reward += temp_reward;
        done |= temp_done;

        info_reward.append(temp_reward);
        info_done.append(temp_done);
    }

    info["reward"] = info_reward;
    info["done"] = info_done;

    return std::make_tuple(py::float_(reward), py::bool_(done), info);
}

void ScrimmageOpenAIEnv::distribute_action(pybind11::object action) {
    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list disc_actions;
        py::list cont_actions;

        if (PyObject_IsInstance(action_space.ptr(), tuple_space)) {
            py::list action_list = action_space.cast<py::list>();
            disc_actions = action_list[0].cast<py::list>();
            cont_actions = action_list[1].cast<py::list>();
        } else if (PyObject_IsInstance(action_space.ptr(), box_space)) {
            if (PyFloat_Check(action.ptr()) || PyInt_Check(action.ptr())) {
                cont_actions.append(action);
            } else {
                cont_actions = action.cast<py::list>();
            }
        } else if (PyObject_IsInstance(action_space.ptr(), discrete_space)) {
            disc_actions.append(action);
        } else {
            disc_actions = action.cast<py::list>();
        }

        int disc_action_idx = 0;
        int cont_action_idx = 0;
        for (auto &a : ext_ctrl_vec_) {
            if (a->action.discrete.size() != a->action_space.discrete_count.size()) {
                a->action.discrete.resize(a->action_space.discrete_count.size());
            }
            if (a->action.continuous.size() != a->action_space.continuous_extrema.size()) {
                a->action.continuous.resize(a->action_space.continuous_extrema.size());
            }
            for (size_t i = 0; i < a->action_space.discrete_count.size(); i++) {
                a->action.discrete[i] = disc_actions[disc_action_idx++].cast<int>();
            }
            for (size_t i = 0; i < a->action_space.continuous_extrema.size(); i++) {
                a->action.continuous[i] = cont_actions[cont_action_idx++].cast<double>();
            }
        }
    } else {

    }
}

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
        .def_readwrite("observation", &ScrimmageOpenAIEnv::observation)
        .def_readwrite("spec", &ScrimmageOpenAIEnv::spec)
        .def_readwrite("metadata", &ScrimmageOpenAIEnv::metadata)
        .def("reset", &ScrimmageOpenAIEnv::reset)
        .def("close", &ScrimmageOpenAIEnv::close)
        .def("seed", &ScrimmageOpenAIEnv::seed)
        .def("render", &ScrimmageOpenAIEnv::render)
        .def_property_readonly("unwrapped", &ScrimmageOpenAIEnv::get_this);
}