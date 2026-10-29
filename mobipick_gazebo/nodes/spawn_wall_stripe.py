#!/usr/bin/env python3
"""Spawn a coloured stripe along the walls of a static world mesh into Gazebo (one static, visual-only SDF model).

In a multi-robot sim each robot's copy of the world gets the stripe in the robot's colour (config/robot_colours.yaml
of mobipick_description), high on the walls so cameras looking at the tables and the base lasers never see it.

Private parameters:
  ~mesh        world mesh: package:// URI, file:// URI or file path (binary or ASCII STL)
  ~offset_x/y  where this copy of the mesh is placed in Gazebo [m] (default 0)
  ~z_low/high  stripe band [m]; default: wall top (highest mesh point) - 0.6 m to wall top - 0.2 m
  ~colour      robot index (1 = first colour) or robot namespace; default: this node's namespace
  ~inset       gap between wall and stripe [m] (default 0.006)
  ~thickness   stripe thickness [m] (default 0.01)
  ~model_name  Gazebo model name (default: <namespace>__wall_stripe)
  ~wait_timeout  seconds to wait for /gazebo/spawn_sdf_model (default 0: wait as long as it takes)
"""
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

from mobipick_gazebo import robot_colours, wall_stripe


def colour_from_param(value, namespace: str):
    """robot_colours entry for an index (int or digit string) or a namespace; None past the table."""
    if isinstance(value, int) or (isinstance(value, str) and value.strip().isdigit()):
        return robot_colours.robot_colour(int(value))
    return robot_colours.robot_colour(str(value or namespace))


def main() -> int:
    rospy.init_node('spawn_wall_stripe')
    namespace = rospy.get_namespace().strip('/')
    mesh = rospy.get_param('~mesh')
    colour = colour_from_param(rospy.get_param('~colour', ''), namespace or 'robot')
    if colour is None:
        rospy.logwarn('spawn_wall_stripe: no robot colour for %r, no stripe spawned', rospy.get_param('~colour', namespace))
        return 0
    model_name = rospy.get_param('~model_name', f'{namespace}__wall_stripe' if namespace else 'wall_stripe')

    path = wall_stripe.resolve_mesh(mesh)
    triangles = wall_stripe.load_stl(path)
    boxes = wall_stripe.stripe_boxes(
        triangles,
        z_low=rospy.get_param('~z_low', None),
        z_high=rospy.get_param('~z_high', None),
        inset=float(rospy.get_param('~inset', wall_stripe.DEFAULT_INSET)),
        thickness=float(rospy.get_param('~thickness', wall_stripe.DEFAULT_THICKNESS)),
    )
    if not boxes:
        rospy.logwarn('spawn_wall_stripe: no wall face of %s covers the stripe band, nothing spawned', path)
        return 0
    sdf = wall_stripe.stripe_sdf(boxes, model_name, colour.gazebo_material)

    service = '/gazebo/spawn_sdf_model'
    timeout = float(rospy.get_param('~wait_timeout', 0.0))
    try:
        rospy.wait_for_service(service, timeout=timeout if timeout > 0 else None)
    except (rospy.ROSException, rospy.ROSInterruptException) as error:
        rospy.logerr('spawn_wall_stripe: %s not available (%s), no %s stripe spawned', service, error, colour.name)
        return 1
    pose = Pose()
    pose.position.x = float(rospy.get_param('~offset_x', 0.0))  # the copy offset goes in the spawn pose only: Gazebo
    pose.position.y = float(rospy.get_param('~offset_y', 0.0))  # combines it with a model <pose>, so that one stays 0
    pose.orientation.w = 1.0
    try:
        response = rospy.ServiceProxy(service, SpawnModel)(model_name, sdf, '', pose, 'world')
    except rospy.ServiceException as error:
        rospy.logerr('spawn_wall_stripe: spawning %s failed: %s', model_name, error)
        return 1
    if not response.success:
        rospy.logerr('spawn_wall_stripe: spawning %s failed: %s', model_name, response.status_message)
        return 1
    rospy.loginfo('spawn_wall_stripe: %s stripe %s with %d pieces on the walls of %s', colour.name, model_name,
                  len(boxes), path.name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
