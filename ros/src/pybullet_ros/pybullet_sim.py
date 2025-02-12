import os

import pybullet as pb
import pybullet_data as pb_data
import rospy
from std_srvs.srv import Trigger, TriggerResponse


class PyBulletSim(object):
    """
    The Simulation class creates the PyBullet physics server and optionally a GUI.

    Available methods (for usage, see documentation at function definition):
        - uid
        - is_alive
        - step
        - add_pybullet_path
        - add_search_path
    """

    def __init__(self, gui=True, gui_options="", start_paused=False):
        """
        Constructor of the Simulation class. This class creates the PyBullet server / GUI and steps the simulation.

        :param gui: Start PyBullet with a GUI
        :param gui_options: GUI options, e.g. "--width=2560 --height=1440"
        :param start_paused: Start simulation paused

        :type gui: bool
        :type gui_options: str
        :type start_paused: bool
        """
        assert isinstance(gui, bool), "[PyBulletSim::init] Parameter 'gui' has an incorrect type."
        assert isinstance(gui_options, str), "[PyBulletSim::init] Parameter 'gui_options' has an incorrect type."
        assert isinstance(start_paused, bool), "[PyBulletSim::init] Parameter 'start_paused' has an incorrect type."

        if gui:
            rospy.loginfo("[PyBulletSim::init] Running PyBullet with GUI")
            rospy.loginfo("-------------------------")
            gui_options = rospy.get_param("~gui_options",
                                          "")  # e.g. to maximize screen: options="--width=2560 --height=1440"
            self._uid = pb.connect(pb.GUI, options=gui_options)
        else:
            rospy.loginfo("[PyBulletSim::init] Running PyBullet without GUI")
            rospy.loginfo("-------------------------")
            self._uid = pb.connect(pb.DIRECT)

        self._simulation_paused = start_paused

        self.add_pybullet_path()

        # setup services to pause/unpause and restart simulation
        rospy.Service("~reset_simulation", Trigger, self.reset_simulation)
        rospy.Service("~pause_simulation", Trigger, self.pause_simulation)
        rospy.Service("~unpause_simulation", Trigger, self.unpause_simulation)

    def __del__(self):
        """
        Disconnect the physics server
        """
        pb.disconnect(self._uid)

    @property
    def uid(self):
        """
        Get UID of physics server

        :rtype: int
        """
        return self._uid

    def is_alive(self):
        """
        Check if the physics server is still connected

        :rtype: bool
        """
        return pb.isConnected(self._uid)

    def is_paused(self):
        """
        Check if the simulation if paused.

        :rtype: bool
        """
        return self._simulation_paused

    def step(self):
        """
        Step the simulation.
        """
        pb.stepSimulation(self._uid)

    def reset_simulation(self, req):
        """
        Reset the simulation.
        """
        self.pause_simulation(Trigger)
        rospy.loginfo("[PyBulletSim::reset_simulation] Resetting simulation.")
        # FIXME calling this will cause the simulation to crash because the joint state plugin cannot get the state
        # pb.resetSimulation(physicsClientId=self._uid)
        self.unpause_simulation(Trigger)
        res = TriggerResponse(True, "Simulation reset")
        return res

    def pause_simulation(self, req):
        """
        Pause the simulation, i.e. disable the stepSimulation() call.
        """
        rospy.loginfo("[PyBulletSim::pause_simulation] Pausing simulation.")
        self._simulation_paused = True
        res = TriggerResponse(True, "Simulation paused")
        return res

    def unpause_simulation(self, req):
        """
        Continue the simulation, i.e. enable the stepSimulation() call.
        """
        rospy.loginfo("[PyBulletSim::unpause_simulation] Unpausing simulation.")
        self._simulation_paused = False
        res = TriggerResponse(True, "Simulation unpaused")
        return res

    @staticmethod
    def add_pybullet_path():
        """
        Adds PyBullets in-built models path to the PyBullet path for easily retrieving the models.
        """
        pb.setAdditionalSearchPath(pb_data.getDataPath())

    @staticmethod
    def add_search_path(path):
        """
        Add the specified directory (absolute path) to PyBullets search path for easily adding models from the path.

        :param path: The absolute path to the directory
        :type path: str

        :return: isdir: Boolean if action was successful
        :rtype: isdir: bool
        """
        assert isinstance(path, str), "[PyBulletSim::add_search_path] Parameter 'path' has an incorrect type."
        if os.path.isdir(path):
            pb.setAdditionalSearchPath(path)
            rospy.loginfo("[PyBulletSim::add_search_path] Added {} to PyBullet path.".format(path))
            return True
        else:
            rospy.logerr(
                "[PyBulletSim::add_search_path] Error adding to PyBullet path! {} not a directory.".format(path))
            return False
