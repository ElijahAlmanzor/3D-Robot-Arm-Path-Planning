#!/usr/bin/env python

#Another node requests for the ik solver - gives cartesian, quaternion 



import rospy

from geometry_msgs.msg import (
    PoseStamped, #what is this message for???
    Pose,
    Point,
    Quaternion,
)
#import the important message types that will be used
#to build the request message for the IK service
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
#So it can communicate with the actual ik_service.py

def ik_service_client(limb, desired_pose, use_advanced_options = False):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK) #connection to the service
    ikreq = SolvePositionIKRequest() #send a request from client to service
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = { 
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=desired_pose.position.x,
                    y=desired_pose.position.y,
                    z=desired_pose.position.z,
                ),
                orientation=Quaternion(
                    x=desired_pose.orientation.x,
                    y=desired_pose.orientation.y,
                    z=desired_pose.orientation.z,
                    w=desired_pose.orientation.w,
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
	#send to position and orientation to the service
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        rospy.loginfo("Running Advanced IK Service Client example.")
        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = [0.1, 0.1, -0.1, -0.1, 0.2, -2, 2]
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [0.1, 0.1, 0.3]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.5)
    else:
        rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
       # rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
       # rospy.loginfo("------------------")
       # rospy.loginfo("Response Message:\n%s", resp)
        return limb_joints
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False

    return True


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, the example will use the default limb
    and call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    rospy.init_node("rsdk_ik_service_client")

    if ik_service_client():
        rospy.loginfo("Simple IK call passed!")
    else:
        rospy.logerr("Simple IK call FAILED")

    if ik_service_client(use_advanced_options=True):
        rospy.loginfo("Advanced IK call passed!")
    else:
        rospy.logerr("Advanced IK call FAILED")


if __name__ == '__main__':
    main()
