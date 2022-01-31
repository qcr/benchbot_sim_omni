import os

map_path = os.environ.get('BENCHBOT_MAP_PATH')
robot_path = os.environ.get('BENCHBOT_ROBOT_PATH')
start_pose = os.environ.get('BENCHBOT_START_POSE')

if __name__ == '__main__':
    # Handle input arguments (env vars due to python.sh bug)
    if start_pose != None:
        start_pose = [float(x) for x in start_pose.split(',')]

    print("Starting Omniverse-powered Isaac Sim simulation with settings:")
    print("\tmap_path:\t%s" % map_path)
    print("\trobot_path:\t%s" % robot_path)
    print("\tstart_pose:\t%s" % start_pose)

    err = False
    if any(x is None for x in [map_path, robot_path, start_pose]):
        print("ERROR: one of map_path, robot_path, or start_pose was none.")
        err = True
    if map_path is None or not os.path.exists(map_path):
        print("ERROR: map_path '%s' does not exist." % map_path)
        err = True
    if robot_path is None or not os.path.exists(robot_path):
        print("ERROR: robot_path '%s' does not exist." % robot_path)
        err = True
    if err:
        quit()

    # Start the simulator
