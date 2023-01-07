# Simple namespace navigation example

'''ros2 launch factory_simulation simple_factory_launch.py'''
This will spawn two turtlebots, but only 'robot1' will have a navigation stack
'''ros2 launch factory_simulation initialize_turtlebots3_launch.py'''
'''ros2 launch factory_simulation navigation2.launch.py use_namespace:=True namespace:=robot1'''
'''ros2 run coordinated_commander example_waypoint_follower'''

# Multi-robot synchronized movement

'''ros2 launch factory_simulation simple_factory_launch.py'''
'''ros2 launch factory_simulation initialize_turtlebots3_launch.py'''
Assuming you are launching from this directory (otherwise, adjust the params_file paths)
'''ros2 launch factory_simulation navigation2.launch.py use_namespace:=True namespace:=robot1 params_file:=params/waffle1'''
'''ros2 launch factory_simulation navigation2.launch.py use_namespace:=True namespace:=robot2 params_file:=params/waffle2'''
'''ros2 run coordinated_commander example_group_waypoint_follower'''
