# Why is this important?

Knowing the difference between static and reserved IP addresses is important because it helps prevent IP addresses conflicts and it also helps with conveying information about how certain devices within a network is setup. In this document, I'll be going over what static and reserved IP addresses are, what the difference between them are, and an exmaple of why it's important to distinguish the difference between the two. 

# What is a static IP?

A static IP is an IP address that a device chooses for itself instead of being assigned an IP from a DHCP server. Basically, when the new host device connects to a network, it tells the network, "I will not accept any other IP address except for this exact one". 

# What is a reserved IP?

A reserved IP is an IP address that the DHCP server saves for a particular device. It kinda sounds like a static IP in a sense that the IP does not change, but the key idea here is the DHCP saves the IP for the device instead of assigning the device a valid IP in the network host range. In order for the DHCP server to know what device to save the IP address for, it needs the **MAC address** (also known as hardware address). The MAC address is an address made specifically for the device (or the device's Network Interface Card to be more accurate) when the device was manufactured. An IP address is (should be) unique for each device in a network, but a MAC address is unique for all devices in the world. 

# What is the difference between a static IP and a reserved IP?

A static IP is an IP address that the host device assigns *itself*. A reserved IP is an IP address that the *DHCP server saves for a specific device*. 

# Example of why this is important

Device A is configured for DHCP which means it is assigned a valid IP address from a DHCP server. It connects to the 10.10.10.0 network and the DHCP assigns Device A with an IP of 10.10.10.20. Device B is not connected to the 10.10.10.0 network yet, but is configured with a static IP setting with its IP as 10.10.10.20. The problem is Device A already has the 10.10.10.20 IP address assigned to it, so what happens when Device B joins the network? Both devices would have the same IP address and there would be an IP conflict. The IP conflict could prevent both devices from properly communicating with the network. For a very large network, this is difficult to troubleshoot since it would look like there is only one device for each IP address. Therefore, it is recommended to use the same method of IP assignment to prevent this. In other words, either have all devices configured for DHCP and if you have a device that needs the same IP address whenever it connects to the network, have the DHCP server reserve the IP address for the device *or* statically set all devices with an IP address. Here is a table to show the pros and cons of each method: 


### DHCP Configuration

|Pros                                         |Cons                                       |
|:-------------------------------------------:|:-----------------------------------------:|
|Easy to have new devices join the network    |If DHCP server fails, whole network crashes|
|Automatic IP management prevents IP conflicts|Yeah, the first one is pretty much it      |
|Reserving IP addresses are easy              |                                           |


### Static Configuration

|Pros                                         |Cons                                                    |
|:-------------------------------------------:|:------------------------------------------------------:|
|IP conflicts are prevented by user management|Have to manually set IP for each device                 |
|Easy to set change IP addresses on the fly   |IP conflicts can still occur if user is not careful     |


