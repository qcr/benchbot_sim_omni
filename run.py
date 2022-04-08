import flask
import numpy as np
import os
import signal
import sys
import time

from builtins import print as bprint
from gevent import event, pywsgi, signal

DEFAULT_POSE = np.array([1, 0, 0, 0, 0, 0, 0])

MAP_PRIM_PATH = '/env'
ROBOT_NAME = 'robot'
ROBOT_PRIM_PATH = '/%s' % ROBOT_NAME
ROBOT_COMPONENTS = {
    'clock': '/ROS_Clock',
    'diff_base': '%s/ROS_DifferentialBase' % ROBOT_PRIM_PATH,
    'lidar': '%s/ROS_Lidar' % ROBOT_PRIM_PATH,
    'rgbd': '%s/ROS_Camera_Stereo_Left' % ROBOT_PRIM_PATH,
    'tf_sensors': '%s/ROS_Carter_Sensors_Broadcaster' % ROBOT_PRIM_PATH,
    'tf': '%s/ROS_Carter_Broadcaster' % ROBOT_PRIM_PATH
}
UPDATE_DELAY_SECS = 3.0


def disable_component(prop_path):
    from omni.kit.commands import execute
    from pxr import Sdf
    print("DISABLING '%s.enabled'" % prop_path)
    execute("ChangeProperty",
            prop_path=Sdf.Path("%s.enabled" % prop_path),
            value=False,
            prev=None)


def print(*args, **kwargs):
    bprint(*args, **kwargs, flush=True)


def tick_component(prop_path):
    from omni.kit.commands import execute
    execute("RosBridgeTickComponent", path=prop_path)


class SimulatorDaemon:

    def __init__(self, port):
        self.address = 'localhost:%s' % port

        self.inst = None
        self.sim = None
        self.sim_i = 0

        self.map_usd = None
        self.robot_usd = None
        self.start_pose = None

        self._robot = None

    def open_usd(self):
        # Bail early if we can't act
        if self.inst is None:
            print("No simulator running. "
                  "Stored environment USD, but not opening.")
            return
        if self.map_usd is None:
            print("No environment USD selected. Returning.")
            return

        # Imports must go after bail early checks pass as they throw errors
        # when called in an "inappropriate state" (no idea what that
        # corresponds to...)
        from omni.isaac.core.utils.stage import (add_reference_to_stage,
                                                 clear_stage, is_stage_loading,
                                                 update_stage)
        # Stop simulation if running
        self.stop_simulation()

        # Update the map
        clear_stage()
        add_reference_to_stage(usd_path=self.map_usd, prim_path=MAP_PRIM_PATH)
        update_stage()

        # Attempt to replace the robot
        self.place_robot()

    def place_robot(self):
        # Bail early if we can't act
        if self.inst is None:
            print("No simulator running. "
                  "Stored robot USD & pose, but not opening.")
            return
        if self.robot_usd is None:
            print("No robot USD selected. Returning.")
            return

        # Imports must go after bail early checks pass as they throw errors
        # when called in an "inappropriate state" (no idea what that
        # corresponds to...)
        from omni.isaac.core.robots import Robot
        from omni.isaac.core.utils.stage import (add_reference_to_stage,
                                                 clear_stage, is_stage_loading,
                                                 update_stage)

        # Stop simulation if running
        self.stop_simulation()

        # Add robot to the environment at the requested pose
        p = DEFAULT_POSE if self.start_pose is None else self.start_pose
        add_reference_to_stage(usd_path=self.robot_usd,
                               prim_path=ROBOT_PRIM_PATH)
        self._robot = Robot(prim_path=ROBOT_PRIM_PATH, name=ROBOT_NAME)
        self._robot.set_world_pose(position=p[4::] * 100, orientation=p[:4])
        update_stage()

        # Disable auto-publishing of all robot components (we'll manually
        # publish at varying frequencies instead)
        for p in ROBOT_COMPONENTS.values():
            disable_component(p)

        # Attempt to start the simulation
        self.start_simulation()

    def run(self):
        f = flask.Flask(__name__)

        @f.route('/', methods=['GET'])
        def __hello():
            return flask.jsonify("Hello, I am the Omniverse Sim Daemon")

        @f.route('/open_environment', methods=['POST'])
        def __open_env():
            r = flask.request.json
            if 'environment' in r:
                self.map_usd = r['environment']
            self.open_usd()
            return flask.jsonify({})

        @f.route('/place_robot', methods=['POST'])
        def __place_robot():
            r = flask.request.json
            if 'robot' in r:
                self.robot_usd = r['robot']
            if 'start_pose' in r:
                # Hard assumption this is a CSV string
                self.start_pose = np.array(
                    [float(x.strip()) for x in r['start_pose'].split(',')])
            self.place_robot()
            return flask.jsonify({})

        @f.route('/restart_sim', methods=['POST'])
        def __restart_sim():
            self.stop_simulation()
            self.start_simulation()
            return flask.jsonify({})

        @f.route('/start', methods=['POST'])
        def __start_inst():
            self.start_instance()
            return flask.jsonify({})

        @f.route('/start_sim', methods=['POST'])
        def __start_sim():
            self.start_simulation()
            return flask.jsonify({})

        @f.route('/stop_sim', methods=['POST'])
        def __stop_sim():
            self.stop_simulation()
            return flask.jsonify({})

        # Start long-running server
        server = pywsgi.WSGIServer(self.address, f)
        evt = event.Event()
        for s in [signal.SIGINT, signal.SIGQUIT, signal.SIGTERM]:
            signal.signal(s, lambda n, frame: evt.set())
        server.start()
        while not evt.is_set():
            evt.wait(0.001)
            self.tick_simulator()

        # Cleanup
        self.stop_instance()

    def start_instance(self):
        if not self.inst is None:
            print("Instance already running. Please /stop first.")
            return
        env = {} if self.map_usd is None else {"open_usd": self.map_usd}

        from omni.isaac.kit import SimulationApp

        # Start the simulator
        self.inst = SimulationApp({
            "renderer": "RayTracedLighting",
            "headless": False,
            **env
        })

        # Import all required modules, and configure application
        from omni.isaac.core.utils.extensions import enable_extension
        from omni.kit.viewport import get_default_viewport_window
        enable_extension("omni.isaac.ros_bridge")

        # Attempt to place the robot if we had a map
        if env:
            self.place_robot()

    def start_simulation(self):
        if self.sim is not None:
            self.stop_simulation()
        if self.inst is None or self.map_usd is None or self.robot_usd is None:
            print("Can't start simulation. Missing some required state.")
            return

        from omni.isaac.core import SimulationContext

        self.sim_i = 0
        self.sim = SimulationContext()
        self.sim.play()

    def stop_instance(self):
        if self.inst is None:
            print("No instance is running to stop.")
            return
        self.stop_simulation()
        self.inst.close()
        self.inst = None

    def stop_simulation(self):
        if self.sim is None:
            print("Skipping. No running simulation to stop")
            return
        if self.inst is None:
            print("Skipping. No running simulator found.")
            return
        self.sim.stop()
        self.sim = None  # TODO maybe could reuse with more guarding logic?

    def tick_simulator(self):
        if self.inst is None:
            return
        if self.sim is None:
            self.inst.update()
            return

        self.sim.step()

        # Tick at 60Hz
        tick_component(ROBOT_COMPONENTS['clock'])

        # Tick at 30Hz
        if self.sim_i % 2 == 0:
            tick_component(ROBOT_COMPONENTS['diff_base'])
            tick_component(ROBOT_COMPONENTS['lidar'])
            tick_component(ROBOT_COMPONENTS['tf'])
            tick_component(ROBOT_COMPONENTS['tf_sensors'])

        # Tick at 10Hz
        if self.sim_i % 6 == 0:
            tick_component(ROBOT_COMPONENTS['rgbd'])
            print("POSE:")
            print(self._robot.get_world_pose())

        self.sim_i += 1


if __name__ == '__main__':
    sd = SimulatorDaemon(port=os.environ.get('PORT'))
    sd.run()
