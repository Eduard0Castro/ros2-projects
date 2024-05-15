from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    rename_number_topic = ("number", "veronica")
    number_node = Node(
        package="number_cpp",
        executable="number_publisher",
        name="Jeronimo",
        remappings=[
            rename_number_topic
        ], 
        parameters=[
            {"number_pub":43},
            {"frequency_pub":5}
        ]
        
    )    

    counter_node = Node(
        package="number_cpp",
        executable="number_counter",
        name="Matias",
        remappings=[
            rename_number_topic,
            ("counter", "Prata")
        ]
    )

    ld.add_action(number_node)
    ld.add_action(counter_node)
    return ld