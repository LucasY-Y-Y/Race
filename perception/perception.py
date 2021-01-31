#!/usr/bin/env python3
# Copied from document:
# perception module for PoPGRI
# The perception module returns all the obstacles within the sensing radius of the vehicle. 
# Each obstacle has a type, id, location, and velocity. 
# The perception can be retrieved by subscribing to the rostopic “local_percep”

# Type: The type of obstacle detected (ie. “pedestrian”, “car”, “building”, etc.)
# ID: An identifier for each obstacle (ie. “pedestrian 1”, etc.). This remains the same across all time steps.
# Location: The location of the center of each obstacle given in [x, y, z] global coordinates
# Velocity: The velocity of the obstacle at the current time step given as [v_x, v_y, v_z].

import carla
import rospy

class PerceptionModule:
    def __init__(self, carla_world, radius=10):
        self.sensing_radius = radius # default ?????
        self.world = carla_world
        self.vehicle = None
    # find ego vehicle
    # reference: 
    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == 'ego_vehicle':
                self.vehicle = actor
                break
    # return all the obstacles within the sensing radius of the vehicle
    def get_all_obstacles_within_range(self):
        # get every actor on stage
        if self.vehicle == None:
            rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        all_actors = self.world.get_actors()
        radius = self.sensing_radius
        filtered_obstacles = []
        for actor in all_actors:
            # get actor's location
            cur_loc = actor.get_location()
            # determine whether actor is within the radius
            if vehicle.distance(cur_loc) <= radius:
                # we need to throw out actors such as camera
                # types we need: vehicle, walkers, Traffic signs and traffic lights
                # reference: https://github.com/carla-simulator/carla/blob/master/PythonAPI/carla/scene_layout.py
                if 'vehicle' in actor.type_id:
                    filtered_obstacles.append(actor)
                elif 'walker' in actor.type_id:
                    filtered_obstacles.append(actor)
                elif 'static.prop' in actor.type_id:
                    filtered_obstacles.append(actor)
        return filtered_obstacles


    # TODO: draw bounding_box ???
    # reference: https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/client_bounding_boxes.py
    # def draw_bounding_box(self, actor):
    #     box = actor.bounding_box
    #     self.world.debug.draw_box(box, )
    
    def set_radius(self, new_radius):
        self.sensing_radius = new_radius
    
    def get_radius(self):
        return self.sensing_radius
    
    # TODO: lane detection
    # get set of waypoints separated by parameter -- distance -- along the lane
    # reference: https://github.com/carla-simulator/carla/issues/1254
    def get_lane_way_point(self, distance=1.0):
        if self.vehicle == None:
            rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        carla_map = self.world.get_map()
        vehicle_location = vehicle.get_location()
        # get a nearest waypoint
        cur_waypoint = carla_map.get_waypvoint(vehicle_location)
        # return list of waypoints from cur_waypoint to the end of the lane
        return cur_waypoint.next_until_lane_end(distance)

from perception_msg.msg import PerceptionData
# TODO: rostopic publish
def publisher(percep_mod):
    rospy.init_node('perception', anonymous=True)
    pub = rospy.Publisher('obstacles',PerceptionData)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        obs = percep_mod.get_all_obstacles_within_range()
        lp = percep_mod.get_lane_way_point()
        message = PerceptionData()
        message.obstacles = obs
        message.lane_waypoints = lp
        pub.publish(message)
        rate.sleep()


if __name__ == '_main_':
    # reference: 
    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/host", 2000)
    client = carla.Client(host=host, port=port)
    client.set_timeout(2)
    world = client.get_world()
    pm = PerceptionModule(world)
    publisher(pm)
