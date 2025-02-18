# Detect the first Ethernet interface that begins with "e"
ETH_INTERFACE=$(ip -o link show | awk -F': ' '$2 ~ /^en/{print $2}' | head -n 1)

# If no interface is found, exit with an error
if [ -z "$ETH_INTERFACE" ]; then
    echo "Ethernet interface not found automatically:"
    echo "To install, find out the name of your ethernet device using 'ifconfig' or 'ip -o link show'"
    echo "Then install using 'docker build --build-arg ETHERNET_INTERFACE=name_of_ethernet_device -t ros2_humble_synapticon .'"
    exit 1
fi

echo "Detected Ethernet interface: $ETH_INTERFACE"

# Build the Docker image and pass the interface name as a build argument
docker build --build-arg ETHERNET_INTERFACE=$ETH_INTERFACE -t ros2_humble_synapticon .
