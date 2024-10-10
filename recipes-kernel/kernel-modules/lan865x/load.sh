#!/bin/bash

echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor > /dev/null

sudo insmod microchip_t1s.ko
sudo insmod lan865x_t1s.ko

mac_addr_1=04:05:06:01:02:03
ip_addr_1=192.168.5.100
subnet_mask_1=255.255.255.0

mac_addr_2=14:15:16:11:12:13
ip_addr_2=192.168.6.100
subnet_mask_2=255.255.255.0

mac_addr=$(ip -o link | grep ether | awk '{ print $2" "$17 }' | tr '\n' ' ')

read -a strarr <<< "$mac_addr"

for val in "${strarr[@]}";
do
	printf "$val\n"
	mac=$val
	if [[ "$mac" == "$mac_addr_1" ]]
	then
		sudo ip addr add dev $eth $ip_addr_1/$subnet_mask_1
	elif [[ "$mac" == "$mac_addr_2" ]]
	then
		sudo ip addr add dev $eth $ip_addr_2/$subnet_mask_2
	fi
	eth=$val
done
