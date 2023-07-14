import flask
import numpy as np
import os
import signal

from builtins import print as bprint
from gevent import event, pywsgi, signal
from pathlib import Path
from spatialmath import SE3, UnitQuaternion

print("STARTING RUN.PY IN BENCHBOT_SIM_OMNI")

DEFAULT_POSE = np.array([1, 0, 0, 0, 0, 0, 0])

DIRTY_EPSILON_DIST = 1
DIRTY_EPSILON_YAW = 2
DIRTY_FILE = '/tmp/benchbot_dirty'

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


def _dc_tf_to_SE3(tf):
    r = np.array(tf.r)
    return SE3(np.array(tf.p)) * UnitQuaternion(r[3], r[0:3]).SE3()


def _to_SE3(pose):
    return SE3(pose[4::]) * UnitQuaternion(pose[0], pose[1:4]).SE3()


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


class SimulatorDaemon:

    def __init__(self, port):
        self.address = 'localhost:%s' % port

        self.inst = None
        self.sim = None

        self.sim_i = 0
        self.sim_collided = False
        self.sim_dirty = False

        self.map_usd = None
        self.robot_usd = None
        self.start_pose = None

        self._map_usd = None
        self._robot_usd = None
        self._start_pose = None

        self._dc = None
        self._robot = None
        self._robot_dc = None

    def check_dirty(self):
        delta = (_to_SE3(self.start_pose).inv() *
                 _dc_tf_to_SE3(self._dc.get_rigid_body_pose(self._robot_dc)))
        return (np.linalg.norm(delta.t[0:2]) > DIRTY_EPSILON_DIST or
                np.abs(delta.rpy(unit='deg')[2]) > DIRTY_EPSILON_YAW)

    def check_collided(self):
        return False

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
        from omni.isaac.core.utils.stage import open_stage, update_stage

        # Stop simulation if running
        self.stop_simulation()

        # Update the map
        if self.map_usd != self._map_usd:
            self._dc = None
            self._start_pose = None
            self._robot = None
            self._robot_dc = None
            self._robot_usd = None

            open_stage(usd_path=self.map_usd)
            update_stage()
            self._map_usd = self.map_usd
        else:
            print("Skipping map load; already loaded.")

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
                                                 update_stage)

        # Stop simulation if running
        self.stop_simulation()

        # Add robot to the environment at the requested pose
        p = DEFAULT_POSE if self.start_pose is None else self.start_pose
        if self.robot_usd != self._robot_usd:
            add_reference_to_stage(usd_path=self.robot_usd,
                                   prim_path=ROBOT_PRIM_PATH)
            self._robot = Robot(prim_path=ROBOT_PRIM_PATH, name=ROBOT_NAME)
            update_stage()
            self._robot_usd = self.robot_usd
        else:
            print("Skipping robot load; already loaded.")

        if (p != self._start_pose).any():
            self._robot.set_world_pose(position=p[4::],
                                       orientation=p[:4])
            update_stage()
            self._start_pose = p
        else:
            print("Skipping robot move; already at requested pose.")

        # Disable auto-publishing of all robot components (we'll manually
        # publish at varying frequencies instead)
        for p in ROBOT_COMPONENTS.values():
            disable_component(p)

        # Attempt to start the simulation
        self.start_simulation()

    def run(self):
        f = flask.Flask('benchbot_sim_omni')

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
                # Probably should be regexing...
                self.start_pose = np.array([
                    float(x.strip()) for x in r['start_pose'].replace(
                        '[', '').replace(']', '').split(',')
                ])
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

        @f.route('/started', methods=['GET'])
        def __started():
            # TODO note there is a race condition (returns true before a /start
            # job finishes)
            return flask.jsonify({'started': self.inst is not None})

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
        print("STARTING INSTANCE!!")
        if not self.inst is None:
            print("Instance already running. Please /stop first.")
            return
        env = {} if self.map_usd is None else {"open_usd": self.map_usd}

        from omni.isaac.kit import SimulationApp

        # Start the simulator
        self.inst = SimulationApp({
            "width": 1280,
            "height": 720,
            "window_width": 1920,
            "window_height": 1080,
            "renderer": "RayTracedLighting",
            "headless": True,
            "sync_loads": True,
            **env
        })

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self.inst.set_setting("/app/window/drawMouse", True)
        self.inst.set_setting("/app/livestream/proto", "ws")
        ext_manager.set_extension_enabled_immediate("omni.kit.livestream.core", True)
        #ext_manager.set_extension_enabled_immediate("omni.kit.livestream.native", True)

        ## Import all required modules, and configure application
        #from omni.isaac.core.utils.extensions import enable_extension
        #enable_extension("omni.isaac.ros_bridge")

        ## Default Livestream settings
        #self.inst.set_setting("/app/window/drawMouse", True)
        #self.inst.set_setting("/app/livestream/proto", "ws")
        #self.inst.set_setting("/app/livestream/websocket/framerate_limit", 120)
        #self.inst.set_setting("/ngx/enabled", False)

        ## Note: Only one livestream extension can be enabled at a time
        ## Enable Native Livestream extension
        ## Default App: Streaming Client from the Omniverse Launcher
        #enable_extension("omni.kit.livestream.native")

        ## Enable WebSocket Livestream extension
        ## Default URL: http://localhost:8211/streaming/client/
        enable_extension("omni.services.streamclient.websocket")

        ## Enable WebRTC Livestream extension
        ## Default URL: http://localhost:8211/streaming/webrtc-client/
        ## enable_extension("omni.services.streamclient.webrtc")


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
        self.sim_collided = False
        self.sim_dirty = False

        self.sim = SimulationContext()
        self.sim.play()

        from omni.isaac.dynamic_control import _dynamic_control

        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._robot_dc = self._dc.get_articulation_root_body(
            self._dc.get_object(ROBOT_PRIM_PATH))

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
        # Tick simulator steps. Does less now than in 2021.2.1 due to new action graph
        if self.inst is None:
            return
        if self.sim is None:
            self.inst.update()
            return

        self.sim.step()

        # Tick at 10Hz CHECK DIRTY
        if self.sim_i % 6 == 0:
            if not self.sim_dirty:
                self.sim_dirty = self.check_dirty()
                if self.sim_dirty:
                    Path(DIRTY_FILE).touch()

        # Tick at 1Hz CHECK COLLIDED
        if self.sim_i % 60 == 0:
            self.sim_collided = self.check_collided()

        self.sim_i += 1


if __name__ == '__main__':
    print("inside run.py __main__")
    sd = SimulatorDaemon(port=os.environ.get('PORT'))
    sd.run()
