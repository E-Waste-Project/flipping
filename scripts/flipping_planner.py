#!/usr/bin/env python
from struct import pack
import rospy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32, String
import actionlib
from cut_contour.msg import MoveStraightAction, MoveStraightActionGoal
from cut_contour.msg import MeasureZAction, MeasureZActionGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cut_contour.robot_helpers import TransformServices, sample_from_func, MotionServices
from math import sqrt, pi, fabs
from copy import deepcopy


class FlippingPlanner():
    def __init__(self, flip_dist=0.00):
        self.flip_point = None
        self.arc_center_point = None
        self.flip_radius = None
        self.flip_dist = flip_dist
        rospy.Subscriber("/flip_xyz", PoseArray, self.old_way)
        #rospy.Subscriber("/px_to_xyz", PoseArray, self.old_way)
        self.measure_client = actionlib.SimpleActionClient(
            'measure_z', MeasureZAction)
        self.move_client = actionlib.SimpleActionClient(
            'move_straight', MoveStraightAction)
        self.flipping_ms = MotionServices("flipping")
        self.camera_ms = MotionServices("camera")
        self.transform_services = TransformServices()
        self.stepper_pub = rospy.Publisher("/pc_to_arduino", Int32, queue_size=1)
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)
        
    def old_way(self, flip_data_msg):
        self.flipping_ms.change_tool_status('Clamp_Off', status = 0)
        rospy.sleep(1)
        self.flipping_ms.change_tool_status('Clamp_On', status = 1)
        
        print("recieved Flip Data")
        trans_flip_data = self.transform_services.transform_poses(
            "base_link", "calibrated_frame", flip_data_msg)
        self.laptop_center = trans_flip_data.poses[0]
        self.flip_point = trans_flip_data.poses[1]
        self.arc_center_point = trans_flip_data.poses[2]
                
        # go to flip pose
        self.flipping_ms.move_group.set_named_target("pre_flipping_pose")
        self.flipping_ms.move_group.go()
        rospy.sleep(1)
        self.flipping_ms.move_group.set_named_target("flipping_pose")
        self.flipping_ms.move_group.go()
        rospy.sleep(1)
        
        # measure z
        poses = PoseArray()
        flip_pose = self.transform_services.lookup_transform("base_link", "flipping_upper")
        poses.poses.append(deepcopy(flip_pose))
        poses.poses[0].position.z -= 0.06
        self.flipping_ms.move_to_touch(poses=poses, axis='y', force_thresh=2)
        rospy.loginfo("Touched !!!!!")
        rospy.sleep(1)
        
        # move slightly upwards to avoid friction.
        poses = PoseArray()
        flip_pose = self.transform_services.lookup_transform("base_link", "flipping_upper")
        flip_pose.position.z += 0.0005
        poses.poses.append(flip_pose)
        self.flipping_ms.move_straight(poses)
        rospy.loginfo("Touched !!!!!")
        rospy.sleep(1)
        
        # extend the stepper
        self.stepper_pub.publish(Int32(data=1))
        
        # wait for the stepper to finish extending
        rospy.wait_for_message("/arduino_to_pc", Int32)
        
        # get flip point by touching laptop
        poses = PoseArray()
        flip_pose = self.transform_services.lookup_transform("base_link", "flipping_upper")
        flip_pose.position.x += 0.2
        poses.poses.append(flip_pose)
        self.flipping_ms.move_to_touch(poses=poses, axis='z', force_thresh=10)
        rospy.loginfo("Touched !!!!!")
        
        # generate plan
        nsteps = 200
        poses = PoseArray()
        self.flip_point = self.transform_services.lookup_transform("base_link", "flipping_upper")
        self.flip_point.position.x -= 0.005
        # self.flip_radius = fabs(self.arc_center_point.position.x - self.flip_point.position.x)
        self.flip_radius = fabs(self.laptop_center.position.x - self.flip_point.position.x) * 2
        print("RADIUS = ", self.flip_radius)
        z_arr, y_arr = sample_from_func(
            lambda z: self.circle_func(z, self.flip_radius + 0.005, self.flip_radius, 0),
            start = -0.005,
            stop = self.flip_radius + self.flip_dist,
            number_of_points = nsteps)
        print("flip_point = ", self.flip_point.position.x)
        print("center_point = ", self.arc_center_point.position.x)
        # start_orientation = [self.flip_point.orientation.x, self.flip_point.orientation.y,
        #                     self.flip_point.orientation.z, self.flip_point.orientation.w]
        # start_orientation = euler_from_quaternion(start_orientation)
       
        
        plan = PoseArray()
        i = 0
        for y, z in zip(y_arr, z_arr):
            # pose_goal = deepcopy(self.flip_point)
            pose_goal = Pose()
            pose_goal.position.x = 0
            pose_goal.position.y = -y
            pose_goal.position.z = z
            q = quaternion_from_euler(i*(pi/nsteps) * 0.1, 0, 0)
            pose_goal.orientation.x = q[0]
            pose_goal.orientation.y = q[1]
            pose_goal.orientation.z = q[2]
            pose_goal.orientation.w = q[3]
            # new_orientation = list(deepcopy(start_orientation))
            # new_orientation[1] -= i*(pi/nsteps) * 0.4
            # quat = quaternion_from_euler(new_orientation[0], new_orientation[1], new_orientation[2])
            # pose_goal.orientation.x = quat[0]
            # pose_goal.orientation.y = quat[1]
            # pose_goal.orientation.z = quat[2]
            # pose_goal.orientation.w = quat[3]
            plan.poses.append(pose_goal)
            i+=1
        plan = self.transform_services.transform_poses("base_link", "flipping_upper", plan)
        
        # execute first half plan
        result = self.flipping_ms.move_straight(plan, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo("Flipped !!!!!")
        if not result:
            rospy.sleep(5)
        else:
            rospy.sleep(1)
        
        # Make pad return behind wall.
        self.stepper_pub.publish(Int32(data=2))
        rospy.wait_for_message("/arduino_to_pc", Int32)
        
        
        pose = self.transform_services.lookup_transform("base_link", "flipping_upper")
        pose.position.x += 0.06
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.header.frame_id = "base_link"
        result = self.flipping_ms.move_straight(pose_array, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo("Moved Straight !!!!!")
        if not result:
            rospy.sleep(5)
        else:
            rospy.sleep(1) 
        
        
        # go to flip posee
        self.flipping_ms.move_group.set_named_target("pre_flipping_pose")
        self.flipping_ms.move_group.go()
        rospy.sleep(1)
        
        # go to flip posee
        self.camera_ms.move_group.set_named_target("horizontal_camera_pose_3")
        self.camera_ms.move_group.go()
        rospy.sleep(1)
        
        # stepper centering
        self.stepper_pub.publish(Int32(data=3))
        
        # wait for the stepper to finish
        rospy.wait_for_message("/arduino_to_pc", Int32)
        
        self.flipping_ms.change_tool_status('Clamp_Off', status = 1)
        rospy.sleep(1)
        self.flipping_ms.change_tool_status('Clamp_On', status = 0)
        
        done_msg = String()
        done_msg.data = "Done"
        self.done_pub.publish(done_msg)
        
        
        
    def flip_cb(self, flip_data_msg):
        trans_flip_data = self.transform_services.transform_poses("base_link", "calibrated_frame", flip_data_msg)
        self.flip_point = trans_flip_data.poses[0]
        self.arc_center_point = trans_flip_data.poses[1]
        self.flip_radius = self.arc_center_point.position.x - self.flip_point.position.x
        x_arr, z_arr = sample_from_func(self.circle_func, self.flip_point.position.x, self.arc_center_point.position.x + self.flip_dist, 100)
        
        plan = PoseArray()
        for x, z in zip(x_arr, z_arr):
            pose_goal = Pose()
            pose_goal.position.x = x
            pose_goal.position.y = self.flip_point.position.y
            pose_goal.position.z = z
            # TODO: Adjust Quaternion values.
            pose_goal.orientation.x = 0
            pose_goal.orientation.y = 1
            pose_goal.orientation.z = 0
            pose_goal.orientation.w = 0
            plan.poses.append(pose_goal)
        
        measure_plan = PoseArray()
        measure_plan.poses.append(plan.poses[0])
        measure_plan.poses[0].position.x -= 0.005
        measure_plan.poses[0].position.z -= 0.02
        
        self.move_client.wait_for_server()
        goal = MeasureZActionGoal(poses=measure_plan, ref_frame="base_link", vel_scale=1,
                                      acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()

        self.move_client.wait_for_server()
        goal = MoveStraightActionGoal(poses=plan, ref_frame="base_link", vel_scale=1,
                                      acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
    
        return self.move_client.get_result()

    def circle_func(self, x, raduis, cx, cz):
        return sqrt(raduis**2 - (x - cx)**2) + cz
        
if __name__ == "__main__":
    rospy.init_node("Flipping_planner")
    flip_planner = FlippingPlanner()
    rospy.spin()
