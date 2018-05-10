"""Example for running openai in scrimmage.

@file

@section LICENSE

Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)

This file is part of SCRIMMAGE.

  SCRIMMAGE is free software: you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.

@author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
@author Eric Squires <eric.squires@gtri.gatech.edu>
@date 31 July 2017
@version 0.1.0
@brief Brief file description.
@section DESCRIPTION
A Long description goes here.
"""

import xml.etree.ElementTree as ET
import copy

import numpy as np
import gym
import scrimmage

import lvdb

MISSION_FILE = 'rlsimple.xml'
TEMP_MISSION_FILE = '.rlsimple.xml'


def _run_test(version, combine_actors, get_action):
    try:
        env = gym.make(version)
    except gym.error.Error:
        gym.envs.register(
            id=version,
            entry_point='scrimmage.external_control:ScrimmageEnv',
            max_episode_steps=1e9,
            reward_threshold=1e9,
            kwargs={"enable_gui": True,
                    "combine_actors": combine_actors,
                    "timeout": 600,
                    "mission_file": TEMP_MISSION_FILE}
        )
        env = gym.make(version)

    # the observation is the state of the aircraft
    env.reset()
    total_reward = 0
    for i in range(200):
        action = get_action(i)
        reward = env.step(action)[1]
        total_reward += reward

    env.close()
    print("Total Reward: %2.2f" % total_reward)

    return env, total_reward


def _write_temp_mission(x_discrete, ctrl_y, y_discrete, num_actors):
    def _update_autonomy_node(entity_node):
        autonomy_node = entity_node.find('autonomy')
        autonomy_node.attrib['x_discrete'] = str(x_discrete)
        autonomy_node.attrib['y_discrete'] = str(y_discrete)
        autonomy_node.attrib['ctrl_y'] = str(ctrl_y)

    tree = ET.parse(scrimmage.find_mission(MISSION_FILE))
    root = tree.getroot()
    entity_node = root.find('entity')
    _update_autonomy_node(entity_node)

    if num_actors == 2:
        entity_node2 = copy.deepcopy(entity_node)
        _update_autonomy_node(entity_node2)
        color_node = entity_node2.find('color')
        color_node.text = "255 0 0"
        root.append(entity_node)

    tree.write(TEMP_MISSION_FILE)


def test_one_dim_discrete():
    """Open single entity scenario and make sure it banks."""
    def _get_action(i):
        return 1 if i < 100 else 0

    VERSION = 'scrimmage-v0'
    _write_temp_mission(
        x_discrete=True, ctrl_y=False, y_discrete=True, num_actors=1)
    combine_actors = False
    env, total_reward = _run_test(VERSION, combine_actors, _get_action)
    assert isinstance(env.action_space, gym.spaces.Discrete)
    assert isinstance(env.observation_space, gym.spaces.Box)
    assert env.action_space.n == 2
    assert total_reward == 4


def test_two_dim_discrete():
    """Open single entity scenario and make sure it banks."""
    def _get_action(i):
        return np.array([1, 1] if i < 100 else [0, 0], dtype=int)

    VERSION = 'scrimmage-v1'
    _write_temp_mission(
        x_discrete=True, ctrl_y=True, y_discrete=True, num_actors=1)
    combine_actors = False
    env, total_reward = _run_test(VERSION, combine_actors, _get_action)

    assert isinstance(env.action_space, gym.spaces.MultiDiscrete)
    assert isinstance(env.observation_space, gym.spaces.Box)
    assert np.array_equal(env.action_space.nvec, np.array([2, 2], dtype=int))
    assert total_reward == 4


def test_one_dim_continuous():
    """Open single entity scenario and make sure it banks."""
    def _get_action(i):
        return 1.0 if i < 100 else -1.0

    VERSION = 'scrimmage-v2'
    _write_temp_mission(
        x_discrete=False, ctrl_y=False, y_discrete=False, num_actors=1)
    combine_actors = False
    env, total_reward = _run_test(VERSION, combine_actors, _get_action)

    assert isinstance(env.action_space, gym.spaces.Box)
    assert isinstance(env.observation_space, gym.spaces.Box)
    assert total_reward == 4


def test_two_veh_dim_discrete():
    """Open single entity scenario and make sure it banks."""
    def _get_action(i):
        return [1, -1] if i < 100 else [-1, 1]

    VERSION = 'scrimmage-v3'
    _write_temp_mission(
        x_discrete=True, ctrl_y=False, y_discrete=False, num_actors=2)
    combine_actors = True
    env, total_reward = _run_test(VERSION, combine_actors, _get_action)

    assert isinstance(env.action_space, gym.spaces.Box)
    assert isinstance(env.observation_space, gym.spaces.Box)
    assert total_reward == 4


if __name__ == '__main__':
    test_one_dim_discrete()
    test_two_dim_discrete()
    test_one_dim_continuous()
    # test_two_veh_dim_discrete()
