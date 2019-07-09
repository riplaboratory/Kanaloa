# Installing ROS Packages

There are three methods of isntalling a ROS package.  You should follow this order of preference.

 1. Using `apt-get` (preferred)
 2. Using `rosdep` (if an apt-get build is not avaialble)
 3. Installing package and dependencies manually (not recommended)
 
# Using `apt-get`

If the package author has made an apt-get build avaialble, this is the most preferred method.  It will be something to the effect of:

```sudo apt-get install package-name```

Then clone the git repository to your catkin workspace in the `src` directory.  

# Using `rosdep`

Create your catkin workspace.  Navigate to the `src` directory of your catkin workspace.  Then clone the repository to your `src` directory.  It will be something to the effect of:

```cd ~/catkin_ws/src
git clone git-repository-url```

Then

```cd ~/catkin_ws && rosdep install -r --ignore-src --from-paths src```

`rosdep` will search for the necessary dependencies.
