# Delivery bot
A robot to serve the orders in a Restaurant

* To visualize the delivery_bot in rviz and gazebo, launch this command

    ```ros2 launch delivery_bot bringup_launch.py```

* Drive the robot using this command in the terminal

    ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```

* To map the restaurant environment launch this below command, with the same teleop_twist_keyboard node

    ```ros2 launch delivery_bot slam_launch.py```

* Use this command to save the map file after map is generated

    ```ros2 run nav2_map_server map_saver_cli -f map_folder/map_file_name```


* To make the robot go autonomously, use this below command (Note : Don't forget to add the map_filename.yaml file in the navigation_launch.py file)

    ```ros2 launch delivery_bot navigation_launch.py```

    you have to localize the bot in the rviz using `2d_pose_estimate` and then give the `2d_goal`, or you can send the `send_goal` command like this below in another terminal

    ```ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"``