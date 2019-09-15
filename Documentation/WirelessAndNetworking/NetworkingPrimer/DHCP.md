# What is DHCP?

DHCP stands for **D**ynamic **H**ost **C**onfiguration **P**rotocol. DHCP is a type of network configuration where a device manages available IP addresses within a network (I'll be referring to this manager device as the DHCP server). In other words, a DHCP server keeps track of what IP addresses are taken, which IP addresses are available, and assigns available IP addresses to new hosts connecting to the network. 

The protocol works like this: 
1. New host connects to network and sends out a request for an IP address to the network
2. DHCP server offers an available IP address to the new host
3. The new host accepts (or rejects) the IP address offered
4. If the new host accepted the IP address offered, the DHCP server assigns the host the IP address. If the host rejected the offered IP, then the DHCP server offers a new IP address and tries again. 

For a great explanation and visual on how DHCP works, I recommned watching this [video](https://www.youtube.com/watch?v=RUZohsAxPxQ). 


## Example

Let's say there are 10 devices already on a network and they are consuming the IP range of 20.20.20.1-20.20.20.10 (this range gives each device their own unique IP address). A new device wants to connect to the network and sends a request for an IP address to the network. The DHCP server offers an IP, the new device accepts the offered IP, and finally the DHCP server assigns the new device the offered IP address. 
