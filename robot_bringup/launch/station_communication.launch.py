from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    names = ["veronica", "brandao", "matias", "jeronimoo", "prata"]
    robots = list()

    for num, name in enumerate (names):
        robots.append(Node(
            package="cpp_pkg",
            executable="robot_station",
            name= f"robot_{name.lower()}",
            parameters=[
                {"robot_name": name}
            ]
        ))
        ld.add_action(robots[num])

    smartphone = Node(
        package="cpp_pkg",
        executable="smartphone"
    )


    ld.add_action(smartphone)

    return ld



