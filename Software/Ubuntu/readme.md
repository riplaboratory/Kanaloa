# Ubuntu

## Background information
[Unbuntu](https://www.ubuntu.com/) is a popular __distribution__ of a  Linux operating system for x86 systems, which is produced by Canonical.  There are many different distributions Linux operating systems, many of which are intended for different purposes.  One requirement for a Linux operating system is that it is "open-source"; i.e. the source code is openly available.  As a result, all Linux operating systems are free to use and install (though it is common for the developer to ask for compensation for their work).  An alternative operating system can be used for many different purposes, in our lab, we utilize Ubuntu to run Robot Operating System (ROS), which is officially compatible with Ubuntu.  

## Installation options
There are three general methods of installing an alternative operating system:
  1. Clean install on a new computer.
  2. Virtualization (running an operating system in a "virtual machine" on top of another operating system).
  3. Dual-booting

In our lab, we take advantage of all three of these methods.  For computers that are specifc to running robot operations, there is no need for a Windows operating system.  Therefore, these computers only run Ubuntu through a standard installation.

For computers that are used for engineering workflows, a Windows operating system is typically necessary.  In order to run Ubuntu on these computers, we have two options: virtualization and dual booting.  Virtualization involves running an operating system inside of a virtual machine, while another operating system is active.  Modern virtualization using solutions like [VMWare Workstation Player](https://www.vmware.com/products/workstation-player.html) provides excellent performance, and have free versions for non-commercial use; however, for robotics applications, there are two major drawbacks to virtualization: (1) although the performance impact is generally minor, there is a significant performance impact to anything involving graphical rendering, generally eliminating this option for vision applications, and (2) I/O (plugging in sensors to the computer) can become a problem for anything other than USB devices.  

The last option, dual-booting, involves installing two (or more) operating systems on the same computer, and selecting which operating system you want to engage every time you power the computer on (therefore, only one operating system is running at any given time).  For machines that require significant workflows in both Windows and Ubuntu, dual-booting is generally the best option.  Dual-booting can be tricky to setup however, because Windows was not necessarily made to run alongside another opearting system, and Ubuntu simply does not have the same level of support as Windows.  This document outlines some tips for a clean dual boot-install.

## Standard installation instructions 
Standard installation instructions can be found [at this link](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0).  You will need a flash drive (or a writable DVD)/

## Virtualization installation instructions 
Instructions for installing VMWare Workstation player, and then installing Unbuntu through this virtual machine can be found [at this link](http://theholmesoffice.com/installing-ubuntu-in-vmware-player-on-windows/)

## Dual-Booting Installation instructions
The process for dualbooting Ubuntu is identical to the standard installation procedured found [at this link](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0).  The exception this time is that you will need to already have a Windows operating system installed, as well as a flash drive (or a writable DVD).

### Dual-booting tips
1. Make sure Windows is installed *first*.  Windows was not necessarily made to run alongside another operating system, but Unbuntu was made to run alongside Windows.  Therefore, Ubuntu *should* install itself correctly if you install it after Windows is already installed on the computer.
2. If you're a beginner, don't attempt to partition your hard drive(s) before you install Linux.  The default install of Linux will (1) resize (shrink) your Windows partition on your primary (C:) drive, (2) create two new partitions in the unallocated space, and (3) install itself on these new partitions.  This way, you don't have to do anthing except tell the installer how much memory on your hard drive you want to allocate to Ubuntu.  The swap partition will be sized based on the amount of RAM you have on the computer, then on top of that we recommend 20-40 GB (the operating system takes around 8-12 GB, and a full install of Matlab takes around 20 GB if you install it).  Therefore, we generally recommend a total of 60 GB for the Ubuntu install if you can afford the space on your hard drive.  
3. [Read this link on partition schemes](https://www.howtogeek.com/howto/35676/how-to-choose-a-partition-scheme-for-your-linux-pc/) if you're interested in partitioning differently.  
4. Get familiar with your computer BIOS, and how to change boot options.  You will need to know this in case Grub (the Ubuntu bootloader application) not not correctly configured by default, which is often the case.  
5. Disable secure boot in your computer BIOS.
6. Disable fast boot in Windows.  This can be done in your power and sleep settings (you need administrative rights to do this).  
7. Run [boot-repair](https://help.ubuntu.com/community/Boot-Repair) if you're having problems booting.
