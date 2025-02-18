====================
Docker Installation
====================

Install Docker and add the user to the Docker group:

.. code:: bash

    sudo apt update
    sudo apt install -y docker.io
    sudo groupadd docker
    sudo usermod -aG docker $USER

