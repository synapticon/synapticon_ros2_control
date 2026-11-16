#!/bin/bash

# Create virtual ethernet interface for testing
echo "Setting up virtual ethernet interface for testing..."

# Remove existing interface if it exists
sudo ip link delete eno0 2>/dev/null || true

# Create new virtual interface
sudo ip link add eno0 type dummy
sudo ip link set eno0 up
sudo ip addr add 192.168.1.100/24 dev eno0

echo "Virtual ethernet interface 'eno0' created successfully"
