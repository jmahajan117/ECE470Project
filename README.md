# ECE470Project

Pot Bot is a simulation of the UR3 Robot using the Gazebo simulator. Pot Bot simulates a poker dealing robot. The intent is for the bot to deal poker cards to a table of up to 6 players. Players will be able to request more cards if they would like. 

# How to run

This project runs on the ECE470 VM provided by the ECE 470 course staff. The project was developed in this Ubuntu environment using VMWare Workstation 15 Player. 

The virtual machine files can be found here: http://coecsl.ece.illinois.edu/ece470/ECE470VM.zip
You can download the files and run them in the VMWare Workstation player to set up Ubuntu.

Once the VM has been set up, the instructions to run the program are largely the same as those found in Lab 2, which you can find here: http://coecsl.ece.illinois.edu/ece470/Lab2_Manual.pdf

To set up the directory, run the following commands:
```
mkdir -p catkin_jaym2/src
cd catkin_jaym2/src
catkin_init_workspace
cd ..
source devel/setup.bash
```

Copy the necessary files over to the build, src, and devel directories. Build the files by running:
```
catkin_make
```

Once the files have been compiled, you can run:
```
roslaunch ur3_driver ur3_gazebo.launch
```

Open a new terminal, make an executable of the Lab 2 file, and run as follows:
```
cd src/lab2pkg_py
chmod +x lab2_exec.py
cd ..
rosrun lab2pkg_py lab2_spawn.py --simulator True
```

Once this command runs, you have the opportunity to specify a starting location for the deck of cards. Then run:
```
rosrun lab2pkg_py lab2_exec.py --simulator True
```

You can then enter the number of players that will be playing to have the bot start dealing cards. The forward kinematics of the robot will be displayed on the terminal.
