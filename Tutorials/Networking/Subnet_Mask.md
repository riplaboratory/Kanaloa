# Subnets and Subnet Masks

## What is a subnet?

Subnets are typically the first 3 segments of an IP address. The subnet specifies the network a host (or device) is a member of. 
An easy way to think of subnets is to think of a groupd of friends. If you have one group of friends from school and another 
group of friend who you grew up with, the members of both groups don't really know each other---the members of each group only 
know the members within their own groups. The same is true with a subnet; devices or hosts within a subnet can only communicate 
with other devices within the same subnet. There is a way for devices from one subnet to communicate with devices in another 
subnet through a gateway, but I actually do not know enough about gatways to explain it. 

### Example

Device A has an IP of 10.10.10.21 while Device B has an IP of 20.20.20.21. Device A is a member of the 10.10.10.0 subnet while 
device B is a member of the 20.20.20.0 subnet. This means they are not able to communicate with each other since they are in 
different subnets. Device A can only communicate with devices in the IP range of 10.10.10.1-10.10.10.254 (**NOTE: The first and
last IP addresses, 10.10.10.0 and 10.10.10.255, is typically reserved for the network and broadcast addresses.**) Likewise, device can only communicate with devices in the IP range of 20.20.20.1-20.20.20.254. 



## What is a subnet mask?

A subnet mask gives information to how large a network is. A subnet mask of 255.255.255.0 means there 254 available host
addresses in the network. A subnet mask of 255.255.255.128 means there are 126 host addresses available in the network. For the 
most part, you should use 255.255.255.0. When creating a subnet mask, 255.255.255.0 is also interchangable with 24 (I don't 
know the details on why, but it's so common it kinda just stuck to memory). 

### Example

Device A has an IP address of 10.10.10.21 with a subnet mask of 255.255.255.0. This means Device A has a host IP of 21 within
the 10.10.10.0 subnet where the 10.10.10.0 subnet has a host range from 10.10.10.1 - 10.10.10.254. Another way to specifiy
Device A's IP address and subnet mask is to say 10.10.10.21/24. 
