import argparse
import sys
import time
import os
import logging

# Bless this mess
from bosdyn.api import geometry_pb2, world_object_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, recording_pb2
from bosdyn.api.mission import nodes_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2

import bosdyn.client
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
import bosdyn.client.channel
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.recording import GraphNavRecordingServiceClient, NotLocalizedToEndError
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
import bosdyn.client.util
from bosdyn.util import duration_str, format_metric, secs_to_hms
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.world_object import WorldObjectClient
    
####################
# MISSION INTERFACE
####################

class MissionInterface:
    # INIT
    def __init__(self, robot, upload_input, mission_file, download_output):
        self._robot = robot
        self._upload_input = upload_input
        self._download_output = download_output
        self._mission_file = mission_file
        
        self._waypoint_id = 'NONE'
        self._waypoint_list = []
        self._waypoint_commands = []
        self._recording = False
        self._exit_check = None
        self._estop_keepalive = None
        
        # Set in start()
        self._robot_id = None
        self._lease = None
        
        self._init_clients()
    
    def _init_clients(self):
        
        # TODO: Difference between mission file and map directory? do we need both?
        
        if not os.path.isdir(self._upload_input):
            LOGGER.error("Cannot find mission file: " + self._upload_input)
            sys.exit(1)
        
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, "GNClient", 9.0)
        except:
            # Using external estop instead
            self._estop_client = None
            self._estop_endpoint = None
            
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._world_object_client = self._robot.ensure_client(WorldObjectClient.default_service_name)
        
        self._recording_client = self._robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
        self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)
        self._mission_client = robot.ensure_client(bosdyn.mission.client.MissionClient.default_service_name)
    
    def __del__(self):
        self.shutdown()
    
    # MISSION
    
    def start(self):
        self._lease = self._lease_client.acquire()
        if self._lease is None:
            raise Exception("Failed to acquire lease!")
        LOGGER.info("Lease acquired: %s", str(self._lease))
        
        self._robot_id = self._robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup()
        
    def run_mission(self):
        with LeaseKeepAlive(self._lease_client):
            # Start
            self._estop_alive()
            self._power_on()
            
            # Load input graph
            self._upload_graph()
            self._upload_mission()
            
            LOGGER.info("Commanding robot to stand...")
            self._self_right()
            self._countdown(5)
            self._sit()
            self._countdown(5)
            self._stand()
            self._countdown(5)
            LOGGER.info("Robot standing...")
            
            # This clears the graph!!
            self._graph_nav_client.clear_graph()
        
            # Start recording
            self._start_recording()
            
            # # Localize robot
            # localization_error = False
            # if do_localization:
            #     graph = graph_nav_client.download_graph()
            #     robot.logger.info('Localizing robot...')
            #     robot_state = robot_state_client.get_robot_state()
            #     localization = nav_pb2.Localization()

            #     # Attempt to localize using any visible fiducial
            #     graph_nav_client.set_localization(
            #         initial_guess_localization=localization, ko_tform_body=None, max_distance=None,
            #         max_yaw=None,
            #         fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST)
            
            # # Do mission
            # if not localization_error:
            #     self._run_mission(robot, mission_client, lease_client, fail_on_question, args.timeout)
                
            # Wait till mission is done?
            
            # Stop recording
            self._stop_recording()
            
            self._sit()
            
            # Download output graph
            self._download_output()
            
            # Stop
            self._safe_power_off()
            self._estop_kill()
            self.shutdown()
            
    # UPLOAD
    
    def _upload_graph(self):
        if not os.path.isdir(self._upload_input):
            LOGGER.error("Cannot find mission file: " + self._upload_input)
            sys.exit(1)
        
        graph_filename = os.path.join(self._upload_input, "graph")
        LOGGER.info("Loading graph from " + graph_filename)
        
        with open(graph_filename, "rb") as graph_file:
            data = graph_file.read()
            current_graph = map_pb2.Graph()
            current_graph.ParseFromString(data)
            LOGGER.info("Loaded graph has {} waypoints and {} edges".format(
                len(current_graph.waypoints), len(current_graph.edges)))
            
        # Load waypoint snapshots
        current_waypoint_snapshots = dict()
        for waypoint in current_graph.waypoints:
            snapshot_filename = os.path.join(path, "waypoint_snapshots", waypoint.snapshot_id)
            LOGGER.info("Loading waypoint snapshot from " + snapshot_filename)
            
            with open(snapshot_filename, "rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        
        # Load edge snapshots
        current_edge_snapshots = dict()
        for edge in current_graph.edges:
            snapshot_filename = os.path.join(path, "edge_snapshots", edge.snapshot_id)
            LOGGER.info("Loading edge snapshot from " + snapshot_filename)
            
            with open(snapshot_filename, "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        
        # Upload graph to robot
        LOGGER.info("Uploading graph and snapshots to robot...")
        self._graph_nav_client.upload_graph(graph=current_graph, lease=self._lease)
        LOGGER.info("Uploaded graph.")
        
        # Upload snapshots
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = current_waypoint_snapshots[snapshot_id]
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot=waypoint_snapshot, lease=lease)
            LOGGER.info('Uploaded {}'.format(waypoint_snapshot.id))

        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = current_edge_snapshots[snapshot_id]
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot=edge_snapshot, lease=lease)
            LOGGER.info('Uploaded {}'.format(edge_snapshot.id))
    
    def _upload_mission(self):
        LOGGER.info("Loading mission from " + self._mission_file)
        
        with open(self._mission_file, "rb") as mission_file:
            data = mission_file.read()
            mission_proto = nodes_pd2.Node()
            mission_proto.ParseFromString(data)
        
        # Upload mission
        LOGGER.info("Uploading mission to robot...")
        self._mission_client.load_mission(mission_proto, leases=[self._lease])
        LOGGER.info("Uploaded mission.")

    # ACTIONS
    
    # Pause for the given duration IN SECONDS
    def _countdown(self, duration):
        for i in range(duration, 0, -1):
            print(i, end=' ', flush=True)
            time.sleep(1)
        print(0)
    
    def _power_on(self):
        # Turn the robot on
        LOGGER.info("Powering on robot... This may take several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        LOGGER.info("Robot powered on.")
    
    def _safe_power_off(self):
        # Turn the robot off (safely)
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), "Robot power off failed."
        LOGGER.info("Robot safely powered off.")
    
    def _estop_alive(self):
        if self._estop_client is None or self._estop_endpoint is None:
            return

        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
    
    def _estop_kill(self):
        if self._estop_client is None or self._estop_endpoint is None:
            return
        
        self._try_grpc("stopping estop", self._estop_keepalive.stop)
        self._estop_keepalive.shutdown()
        self._estop_keepalive = None
        
    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info("Shutting down LIDARMission.")
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None
    
    def _self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())
        
    def _start_recording(self):
        """Start recording a map."""
        if not self._is_fiducial_visible():
            LOGGER.error("Cannot start recording: No fiducials visible.")
            return
        
        if self._waypoint_id is None:
            LOGGER.error("Cannot start recording: Not localized to waypoint.")
            return
        
        status = self._recording_client.start_recording()
        if status != recording_pb2.StartRecordingResponse.STATUS_OK:
            LOGGER.error("Start recording failed.")
            return
            
        LOGGER.info("Started recording successfully.")
        self._waypoint_list = []
        self._waypoint_commands = []
        self._recording = True
    
    def _stop_recording(self):
        """Stop or pause recording a map."""
        if not self._recording:
            return True
            
        self._recording = False
        self._waypoint_commands += [self._waypoint_id]

        try:
            status = self._recording_client.stop_recording()
            if status != recording_pb2.StopRecordingResponse.STATUS_OK:
                LOGGER.error("Stop recording failed.")
                return False

            LOGGER.info("Stopped recording successfully.")
            return True

        except NotLocalizedToEndError:
            LOGGER.error("Failed to stop recording: Must move to final waypoint before stopping recording.")
            return False
            
    def _relocalize(self):
        """Insert localization node into mission."""
        if not self._recording:
            print('Not recording mission.')
            return False

        LOGGER.info("Adding fiducial localization to mission.")
        self._waypoint_commands += [self._waypoint_id]
        self._waypoint_commands += ['LOCALIZE']
        return True
        
    def _replay_mission(self):
        pass
    
    # HELPERS
    
    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError) as err:
            LOGGER.error("Failed {}: {}".format(desc, err))
            return None
    
    def _start_robot_command(self, desc, command_proto, end_time_secs=None):
        def _start_command():
            self._robot_command_client.robot_command(lease=None, command=command_proto, end_time_secs=end_time_secs)
        self._try_grpc(desc, _start_command)

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state
    
    def _count_visible_fiducials(self):
        """Return number of fiducials visible to robot."""
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(
            object_type=request_fiducials).world_objects
        return len(fiducial_objects)
        
    def _is_fiducial_visible(self):
        """Return True if robot can see fiducial."""
        return self._count_visible_fiducials() > 0
        
    # DOWNLOAD 
    
    def _download_recording(self):
        """Save graph map and mission file."""
        
        if not self._recording:
            LOGGER.error("Error: No mission recorded.")
            return
        
        if not self._stop_recording():
            LOGGER.error("Error while stopping recording.")
            return
        
        # Save graph
        os.mkdir(self._download_output)
        if not self._download_full_graph():
            LOGGER.error("Error while downloading graph.")
            return
        
        # Save mission    
        mission = self._generate_mission()
        os.mkdir(self._download_output + "/missions")
        mission_filepath = self._download_output + "/missions/autogenerated"
        write_mission(mission, mission_filepath)
        
        self._quit()
    
    def _download_full_graph(self):
        """Download the graph and snapshots from the robot."""
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            LOGGER.error("Failed to download the graph.")
            return False

        # Write graph map
        self._write_full_graph(graph)
        LOGGER.info("Graph downloaded with {} waypoints and {} edges".format(
            len(graph.waypoints), len(graph.edges)))

        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints)
        self._download_and_write_edge_snapshots(graph.edges)
        return True
        
    def _write_full_graph(self, graph):
        """Download the graph from robot to the specified, local filepath location."""
        graph_bytes = graph.SerializeToString()
        write_bytes(self._download_output, '/graph', graph_bytes)
        
    def _download_and_write_waypoint_snapshots(self, waypoints):
        """Download the waypoint snapshots from robot to the specified, local filepath location."""
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                LOGGER.error("Failed to download waypoint snapshot: " + waypoint.snapshot_id)
                continue

            write_bytes(self._download_output + '/waypoint_snapshots', '/' + waypoint.snapshot_id, waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            LOGGER.info("Downloaded {} of the total {} waypoint snapshots.".format(num_waypoint_snapshots_downloaded, len(waypoints)))

    def _download_and_write_edge_snapshots(self, edges):
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        for edge in edges:
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                LOGGER.error("Failed to download edge snapshot: " + edge.snapshot_id)
                continue

            write_bytes(self._download_output + '/edge_snapshots', '/' + edge.snapshot_id, edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            LOGGER.info("Downloaded {} of the total {} edge snapshots.".format(num_edge_snapshots_downloaded, len(edges)))
    
    def _generate_mission(self):
        """ Create a mission that visits each waypoint on stored path."""

        LOGGER.info("Mission: " + str(self._waypoint_commands))

        # Create a Sequence that visits all the waypoints.
        sequence = nodes_pb2.Sequence()
        sequence.children.add().CopyFrom(self._make_localize_node())

        for waypoint_id in self._waypoint_commands:
            if waypoint_id == 'LOCALIZE':
                sequence.children.add().CopyFrom(self._make_localize_node())
            else:
                sequence.children.add().CopyFrom(self._make_goto_node(waypoint_id))

        # Return a Node with the Sequence.
        ret = nodes_pb2.Node()
        ret.name = "Visit %d goals" % len(self._waypoint_commands)
        ret.impl.Pack(sequence)
        return ret
    
    def _make_goto_node(self, waypoint_id):
        """ Create a leaf node that will go to the waypoint. """
        ret = nodes_pb2.Node()
        ret.name = "goto %s" % waypoint_id
        vel_limit = NAV_VELOCITY_LIMITS
        tolerance = graph_nav_pb2.TravelParams.TOLERANCE_DEFAULT
        travel_params = graph_nav_pb2.TravelParams(velocity_limit=vel_limit, feature_quality_tolerance=tolerance)

        impl = nodes_pb2.BosdynNavigateTo(travel_params=travel_params)
        impl.destination_waypoint_id = waypoint_id
        ret.impl.Pack(impl)
        return ret
    
    def _make_localize_node(self):
        """Make localization node."""
        loc = nodes_pb2.Node()
        loc.name = "localize robot"

        impl = nodes_pb2.BosdynGraphNavLocalize()
        impl.localization_request.fiducial_init = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST

        loc.impl.Pack(impl)
        return loc

######################
# MORE DOWNLOAD STUFF
######################

def write_bytes(filepath, filename, data):
    """Write data to a file."""
    os.makedirs(filepath, exist_ok=True)
    with open(filepath + filename, 'wb+') as f:
        f.write(data)
        f.close()


def write_mission(mission, filename):
    """ Write a mission to disk. """
    open(filename, 'wb').write(mission.SerializeToString())
    
########
# SETUP
########

LOGGER = logging.getLogger()

def parse_arguments(argv):
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    
    # Can add more command line arguments here
    parser.add_argument("--output", help="Output directory for graph map and mission file.")
    parser.add_argument('--input', help='Input path to map directory'
    parser.add_argument('--mission', dest='mission_file', help='Optional path to mission file, defaults to autogenerated one inside input directory')
    
    return parser.parse_args(argv)
    
def setup_logging(is_verbose):
    LOGGER.setLevel(logging.DEBUG)
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    
    file_handler = logging.FileHandler('log_output.log')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_formatter)
    LOGGER.addHandler(file_handler)
    
    if is_verbose:
        stream_level = logging.DEBUG
    else:
        stream_level = logging.INFO
    
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(stream_level)
    stream_handler.setFormatter(log_formatter)
    LOGGER.addHandler(stream_handler)
    return stream_handler

def init_robot(hostname, username, password):
    sdk = bosdyn.client.create_standard_sdk("LIDARMissionClient")
    robot = sdk.create_robot(config.hostname)
    robot.authenticate(config.username, config.password)
    robot.time_sync.wait_for_sync()
    return robot

def main(argv):
    options = parse_arguments(argv)
    stream_handler = setup_logging(options.verbose)
    robot = None
    
    if not options.input or not options.output:
        raise Exception("Must specify input and output paths")
        
    if not options.mission_file:
        options.mission_file = os.path.join(options.input, "missions", "autogenerated")
        
    
    try:
        robot = init_robot(options.hostname, options.username, options.password)
    except RpcError as e:
        LOGGER.error("Failed to communicate with robot: %s" % e)
        return False
    
    mission_interface = MissionInterface(robot, options.input, options.mission_file, options.output)
    try:
        mission_interface.start()
    except Exception as e:
        LOGGER.error("Failed to initialize robot: %s" % e)
        return False

    try:
        mission_interface.run_mission()
    except Exception as e:
        logger = bosdyn.client.util.get_logger()
        logger.error("LIDARMission threw a runtime exception: %r" % e)
        return False
        
    return True

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
