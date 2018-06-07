"""Provide OpenAI interface for SCRIMMAGE."""
from __future__ import print_function
import threading
import warnings
import subprocess
import collections
import time
import os
import signal
import sys
import xml.etree.ElementTree as ET
import importlib
import argparse
import platform

import lvdb
from concurrent import futures
import numpy as np
import grpc

import gym
import gym.spaces
from gym.utils import seeding
if __name__ == "__main__":
    from scrimmage.proto import ExternalControl_pb2, ExternalControl_pb2_grpc
else:
    from .proto import ExternalControl_pb2, ExternalControl_pb2_grpc

if sys.version[0] == '2':
    import Queue as queue
else:
    import queue as queue


class ServerThread(threading.Thread):
    """Start SCRIMMAGE ExternalControl GRPC Service."""

    def __init__(self, queues, address, timeout, max_workers=10):
        """Initialize variables."""
        super(ServerThread, self).__init__()
        self.queues = queues
        self.address = address
        self.max_workers = max_workers
        self.timeout = timeout
        self.stop = False
        self.stopped = True

    def run(self):
        """Start SCRIMMAGE ExternalControl GRPC Service."""
        server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers))

        ExternalControl_pb2_grpc.add_ExternalControlServicer_to_server(
            ExternalControl(self.queues, self.timeout), server)
        server.add_insecure_port(self.address)
        self.stopped = False
        server.start()

        try:
            while not self.stop:
                time.sleep(0.1)
            server.stop(0)
        except KeyboardInterrupt:
            server.stop(0)

        self.stopped = True


class ScrimmageEnv(gym.Env):
    """OpenAI implementation for SCRIMMAGE."""

    metadata = {'render.modes': ['human']}

    def __init__(self, enable_gui, mission_file,
                 combine_actors=True,
                 port_offset=1,
                 address="localhost:50051",
                 timeout=600,
                 global_sensor=False,
                 gdb_args=""):
        """Create queues for multi-threading."""
        signal.signal(signal.SIGINT, self._signal_handler)

        self.enable_gui = enable_gui
        self.global_sensor = global_sensor
        self.mission_file = mission_file
        self.combine_actors = combine_actors
        self.address = address
        self.gdb_args = gdb_args
        self.timeout = timeout
        self.port_offset = port_offset

        self.rng = None
        self.seed = self._seed(None)

        queue_names = ['env', 'action', 'action_response']

        ip, port = address.split(":")
        self.queues = []
        port = int(port)
        address = ip + ":" + str(port)
        self.queues = {s: queue.Queue() for s in queue_names}
        self.server_thread = ServerThread(self.queues, address, self.timeout)
        self.server_thread.start()

        # startup headless version of scrimmage to get the environment
        self.scrimmage_process = self._start_scrimmage(False, True)

        envs = self.queues['env'].get(timeout=self.timeout)
        self.num_actors = len(envs.envs)

        self._clear_queues()
        self._terminate_scrimmage()
        self._clear_queues()

        if self.num_actors == 1:
            self.action_space, self.observation_space, self.reward_range = \
                self._create_spaces(envs.envs[0])
        else:
            if self.combine_actors:
                self._env = ExternalControl_pb2.Environment()
                for i, e in enumerate(envs.envs):
                    for p in e.action_spaces.params:
                        self._env.action_spaces.params.extend([p])
                    if not self.global_sensor or i == 0:
                        for p in e.observation_spaces.params:
                            self._env.observation_spaces.params.extend([p])
                self.reward_range = \
                    (sum([e.min_reward for e in envs.envs]),
                     sum([e.max_reward for e in envs.envs]))
                self.action_space, self.observation_space = \
                    self._create_spaces(self._env)[:2]
            else:
                spaces = [self._create_spaces(e) for e in envs.envs]
                action_space, observation_space, reward_range = zip(*spaces)
                self.action_space = gym.spaces.Tuple(action_space)
                self.observation_space = gym.spaces.Tuple(observation_space)
                min_rewards, max_rewards = zip(*reward_range)
                self.reward_range = (min(min_rewards), max(max_rewards))

    def _create_spaces(self, environment):
        try:
            action_space = \
                _create_tuple_space(environment.action_spaces)
            observation_space = \
                _create_tuple_space(environment.observation_spaces)
            reward_range = (environment.min_reward, environment.max_reward)
            return action_space, observation_space, reward_range
        except AssertionError:
            print(('scrimmage external_control.py: calling terminate '
                   'from __init__ due to env problem'))
            self.close()
            raise

    def _signal_handler(self, signum, frame):
        """Exit cleanly <ctrl-c> (i.e., kill the subprocesses)."""
        print("scrimmage external_control.py: exiting scrimmage from sigint")
        self.close()
        sys.exit(0)

    def reset(self):
        """Restart scrimmage and return result."""
        self._clear_queues()
        self._terminate_scrimmage()
        self._clear_queues()

        self.scrimmage_process = \
            self._start_scrimmage(self.enable_gui, False)

        return self._return_action_result()[0]

    def step(self, action):
        """Send action to SCRIMMAGE and return result."""
        actions_pb = ExternalControl_pb2.Actions()

        def _add_action(space, a):
            if not isinstance(a, collections.Iterable):
                a = [a]

            if isinstance(space, gym.spaces.Box):
                action_pb = \
                    ExternalControl_pb2.Action(continuous=a, done=False)
            else:
                action_pb = \
                    ExternalControl_pb2.Action(discrete=a, done=False)

            actions_pb.actions.extend([action_pb])

        if self.num_actors == 1:
            _add_action(self.action_space, action)
        elif self.combine_actors:
            for a in action:
                _add_action(self.action_space, a)
        else:
            for i, a in enumerate(action):
                _add_action(self.action_space.spaces[i], a)

        self.queues['action'].put(actions_pb)
        return self._return_action_result()

    def _render(self, mode='human', close=False):
        """Ignores a render call but avoids an exception.

        If a user wants the environment rendered then the user should
        pass enable_gui=True as a kwarg
        """
        pass

    def _seed(self, seed=None):
        self.rng, seed = seeding.np_random(seed)
        return [seed]

    def _clear_queues(self):
        for key in self.queues:
            _clear_queue(self.queues[key])

    def _start_scrimmage(self, enable_gui, disable_output):

        def _update_node(base_node, tag, text):
            node = base_node.find(tag)
            if node is None:
                base_node.append(ET.Element(tag))
                node = base_node.find(tag)
            node.text = text

        port = self.address.split(":")[1]
        tree = ET.parse(self.mission_file)
        root = tree.getroot()

        # set the seed using this class' random number generator
        _update_node(root, "seed", str(self.rng.randint(0, 2**32 - 1)))

        for nd in root.findall('entity_interaction'):
            if nd.text == 'ExternalControlInteraction':
                nd.attrib['server_address'] = self.address
                nd.attrib['timeout'] = str(self.timeout)
                break

        # enable gui
        run_node = root.find('run')
        run_node.attrib['enable_gui'] = str(enable_gui)
        if not bool(enable_gui):
            run_node.attrib['time_warp'] = "0"

        # disable output
        if disable_output:
            _update_node(root, "output_type", " ")
            _update_node(root, "display_progress", "false")

        self.temp_mission_file = \
            "." + port + platform.node() + os.path.basename(self.mission_file)

        # print('temp mission file is ' + self.temp_mission_file)
        tree.write(self.temp_mission_file)
        if self.gdb_args:
            cmd = self.gdb_args.split(" ") + \
                ["scrimmage", self.temp_mission_file]
        else:
            cmd = ["scrimmage", self.temp_mission_file]
        cmd = ["scrimmage", self.temp_mission_file]
        # cmd = ["valgrind", "--tool=callgrind", "scrimmage", self.temp_mission_file]
        # cmd = ['gdb', '-x', '~/scrimmage/.gdbinit', '-f', 'scrimmage']
        # print(cmd)
        return subprocess.Popen(cmd)

    def close(self):
        """Cleanup spawned threads and subprocesses.

        The thread manages a GRPC server to communicate with scrimmage.  The
        subprocess is the actual scrimmage instance.  This method needs to be
        called in order to make sure a python instance exits cleanly.
        """
        self._terminate_scrimmage()
        self.server_thread.stop = True
        while not self.server_thread.stopped:
            time.sleep(0.1)

    def _return_action_result(self):
        info = {}
        try:
            res = self.queues['action_response'].get(timeout=self.timeout)
        except queue.Empty:
            print('Scrimmage Environment: error getting action result')
            res = ExternalControl_pb2.ActionResults(done=True)
            info['scrimmage_err'] = ""

        try:
            res.action_results
        except AttributeError:
            return None, None, True, {}

        if self.num_actors == 1:
            obs = np.array(res.action_results[0].observations.value)
            rew = res.action_results[0].reward
            done = res.done or res.action_results[0].done
        else:
            info['rewards'] = [r.reward for r in res.action_results]
            rew = sum(info['rewards'])

            info['done'] = [r.done for r in res.action_results]
            done = res.done or any(info['done'])

            if self.combine_actors:
                if self.global_sensor:
                    obs = np.array(
                        [v for v in res.action_results[0].observations.value])
                else:
                    obs = np.array([v for r in res.action_results
                                    for v in r.observations.value])
            else:
                obs = [np.array(r.observations.value)
                       for r in res.action_results]

        return obs, rew, done, info

    def _terminate_scrimmage(self):
        """Terminates scrimmage instance held by the class.

        given how sigints are handled by scrimmage, we need to
        shutdown the network to the autonomy in addition to sending a
        sigint.
        """
        self.queues['action'].put(ExternalControl_pb2.Action(done=True))
        if self.scrimmage_process is None:
            return

        self.scrimmage_process.poll()
        while (self.scrimmage_process is not None and
               self.scrimmage_process.returncode is None):

            self.queues['action'].put(ExternalControl_pb2.Actions(done=True))
            time.sleep(0.1)
            self.scrimmage_process.poll()


class ExternalControl(ExternalControl_pb2_grpc.ExternalControlServicer):
    """GRPC Service to communicate with SCRIMMAGE Autonomy."""

    def __init__(self, queues, timeout):
        """Receive queues for multi-threading."""
        self.queues = queues
        self.timeout = timeout

    def SendEnvironments(self, env, context):
        """Receive Environment proto and send back an action."""
        self.queues['env'].put(env)
        return ExternalControl_pb2.Empty()

    def SendActionResults(self, action_results, context):
        """Receive ActionResult proto and send back an action."""
        self.queues['action_response'].put(action_results)
        try:
            action = self.queues['action'].get(timeout=self.timeout)
        except queue.Empty:
            action = ExternalControl_pb2.Action(done=True)
        return action


def _create_tuple_space(space_params):
    discrete_extrema = []
    continuous_extrema = []

    for param in space_params.params:
        if param.discrete:
            # openai discrete spaces take in the number of inputs, not the
            # maximum
            maximums = [int(m + 1) for m in list(param.maximum)]
            if param.num_dims != 1 and len(maximums) == 1:
                discrete_extrema += param.num_dims * [maximums[0]]
            else:
                discrete_extrema += maximums
        else:
            if param.num_dims != 1 and len(param.maximum) == 1:
                # use same min/max for all dims
                continuous_extrema += \
                    param.num_dims * [[param.minimum[0], param.maximum[0]]]
            else:
                # each min/max is specified individually
                continuous_extrema += \
                    zip(list(param.minimum), list(param.maximum))

    if len(discrete_extrema) == 1:
        discrete_space = gym.spaces.Discrete(discrete_extrema[0])
    else:
        discrete_space = gym.spaces.MultiDiscrete(discrete_extrema)

    if continuous_extrema:
        low, high = zip(*continuous_extrema)
        continuous_space = gym.spaces.Box(
            np.array(low), np.array(high), dtype=np.float32)

    if discrete_extrema and continuous_extrema:
        return gym.spaces.Tuple((discrete_space, continuous_space))
    elif discrete_extrema:
        return discrete_space
    elif continuous_extrema:
        return continuous_space
    else:
        raise NotImplementedError(
            "received a space with no discrete or continuous components")


def _clear_queue(q):
    try:
        while True:
            q.get(False)
    except queue.Empty:
        pass
