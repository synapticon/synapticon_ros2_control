================================
Synapticon Package Installation
================================

Copy the ``Dockerfile`` and ``synapticon_ros2_controller_build.sh`` script from the ``ros2_humble_docker`` to the same folder on your machine. With the following command, you will run the bash script that will automatically install ROS2 and the package into the Docker image, detect the hardware, and replace the ethernet adapter name in the necessary files. In case the script does not detect your hardware properly, you will need to follow the instructions from its output to build the Docker image. In the folder where you placed the two aforementioned files, run:

.. code:: bash

    bash ./synapticon_ros2_controller_build.sh

To allow Docker containers to output the screen on your system (this is required for RViZ), execute this on the host system:

.. code:: bash

    xhost +

For the first execution of the program, we build a container named ``ros2_container`` from the image ``ros2_humble_synapticon`` that we generated using the bash script from the Dockerfile with the following command (this command is executed only once):

.. code:: bash

    docker run -it -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket -v /tmp/.X11-unix:/tmp/.X11-unix --ipc=host -e DISPLAY=$DISPLAY --network=host --env QT_X11_NO_MITSHM=1 --privileged --name ros2_container ros2_humble_synapticon

Now we have our container running. Each other time, we start the container using:

.. code:: bash

    docker start ros2_container

To open a new terminal in the running container, use:

.. code:: bash

    docker exec -it ros2_container bash

and, once it opens, source the ROS2 environment using:

.. code:: bash

    source /root/.bashrc

To check if the master could be run and if the slaves are found, in the container terminal execute:

.. code:: bash

    ./install/synapticon_ros2_control/bin/torque_control_executable

Before running other scripts, stop this one by pressing CTRL+C (or wait, it will shut down automatically after a while).

