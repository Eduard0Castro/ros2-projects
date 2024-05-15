from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_spawner = Node(
        package="move_turtle",
        executable="spawner_turtle",
        parameters=[
            {"frequency": 1.0}
        ]
    )

    turtle_controller = Node(
        package="move_turtle",
        executable="controller"
    )

    ld.add_action(turtlesim)
    ld.add_action(turtle_spawner)
    ld.add_action(turtle_controller)
    return ld