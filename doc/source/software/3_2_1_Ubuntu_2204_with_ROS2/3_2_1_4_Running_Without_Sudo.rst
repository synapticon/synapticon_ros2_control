================================
Running Without Sudo (Optional)
================================

If you want to run the example without using ``sudo``, you need
to create:

.. code:: bash

    sudo touch /etc/systemd/system/ros2_control_node.service

and use a text editor to paste in that file the following:

.. code:: ini

    [Unit]
    Description=Launch ros2_control_node with socket permissions

    [Service]
    Type=simple
    User=YOUR_USER
    ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash; source /home/YOUR_USER/.bashrc; source /home/YOUR_USER/ros2_ws/install/setup.bash'
    # Write the user environment to file, for debugging
    #ExecStartPre=/bin/bash -c 'env > /home/YOUR_USER/Documents/ros_env_before_start.txt'

    # This is essentially a copy of my normal user env
    Environment="AMENT_PREFIX_PATH=/home/YOUR_USER/ros2_ws/install/synapticon_ros2_control:/opt/ros/humble"
    Environment="HOME=/home/YOUR_USER"
    Environment="LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib"
    Environment="PATH=/opt/ros/humble/bin:/usr/lib/ccache:/home/your_user/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin"
    Environment="PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
    Environment="ROS_DISTRO=humble"
    Environment="ROS_DOMAIN_ID=1"
    Environment="ROS_PYTHON_VERSION=3"
    Environment="ROS_VERSION=2"
    Environment="ROSCONSOLE_FORMAT=[${severity}] - ${node}: [${time}] ${message}"
    Environment="USER=YOUR_USER"
    Environment="USERNAME=YOUR_USER"

    ExecStart=/opt/ros/humble/bin/ros2 launch synapticon_ros2_control elevated_permissions_X_dof.launch.py
    AmbientCapabilities=CAP_NET_RAW

    [Install]
    WantedBy=multi-user.target

After pasting, do not forget to replace ``YOUR_USER`` with your
username and ``X_dof`` with 1 or 2 in the line specifying which launch file needs to be executed. Save the file, restart the daemon:

.. code:: bash

    sudo systemctl daemon-reload

and start the service:

.. code:: bash

    sudo systemctl restart ros2_control_node.service

If you want to check the service status and see the ROS console
logging:

.. code:: bash

    sudo systemctl status ros2_control_node.service

Now, the example can be run by these two commands:

.. code:: bash

    sudo systemctl restart ros2_control_node.service

and, if running the demo with one motor:

.. code:: bash

    ros2 launch synapticon_ros2_control single_dof.launch.py

If you are running the demo with two motors:

.. code:: bash

    ros2 launch synapticon_ros2_control two_dof.launch.py           

Changing the controllers and publishing the desired
position/velocity/torque can now be executed without ``sudo``. To
stop the ``ros2_control_node``:

.. code:: bash

    sudo systemctl stop ros2_control_node.service

