# GENG5508 Playing Connect 4 with Baxter

Our task is to implement a program that lets [Baxter](http://www.rethinkrobotics.com/baxter/) play Connect 4 on a standard playing board. While code for this [already exists](http://sdk.rethinkrobotics.com/wiki/Connect_Four_Demo), we are recreating it from scratch.

**Unit:** GENG5508 Robotics at UWA

**Team Name:** HAL 9001 (Patrick Kenworthy, Luca Trimboli, Brett Leask, and Alex Arnold)

**Due Date:** Week 13, Semester 2, 2017


## Getting Started

The next section will explain how to get everything up and running.

### Prerequisites

  *  A Baxter machine or simulation environment
  *  A development machine with ROS installed
    *  We used ROS Indigo in Ubuntu 14.04 under VirtualBox
  *  A Connect 4 game board
  *  Customised Baxter gripper (to pick up game pieces)
  *  A game piece feeder (for Baxter to grab game pieces from)

### Installing

  1.  Clone or download the repository to your ROS `/src` folder
    *  Default location is `~/ros_ws/src`
  2.  Connect to Baxter (or a simulator) using `baxter.sh`
  3.  Start the program using `rosrun`:

  `$ rosrun geng5508_baxter_connect4 play.py -l right -c red -d 4`
    *  The `-l` option specifies which limb will be used to pick up and place game pieces (either `left` or `right`)
    *  The `-c` option specifies the colour game piece that Baxter is playing with (either `red` or `yellow`)
    * The `-d` option specifies the difficulty/recursion depth of the Connect 4 AI. The larger the value the better Baxter will play, but the longer it will take to calculate a move.

## Authors

  *  Patrick Kenworthy (21148951)
  *  Luca Trimboli (21296303)
  *  Brett Leask (21504732)
  *  Alex Arnold (21304455)

## License

This project is licensed under the MIT License - see LICENSE for details.

## Acknowledgements

 *  Kai Li Lim, for assisting us throughout the project
