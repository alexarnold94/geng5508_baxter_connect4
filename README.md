# GENG5508 Playing Connect 4 with Baxter

Our task is to extend an existing project that allows [Baxter](http://www.rethinkrobotics.com/baxter/) to play Connect 4 on a standard playing board. Using computer vision techniques, the solution was made more robust and easier to play.

**Unit:** GENG5508 Robotics at UWA

**Team Name:** HAL 9001 (Patrick Kenworthy, Luca Trimboli, Brett Leask, and Alex Arnold)

**Due Date:** Friday 2nd November, 2017


## Getting Started

The next section will explain how to get everything up and running.

### Prerequisites

  *  A Baxter machine or simulation environment
  *  A development machine with ROS installed (We used ROS Indigo in Ubuntu 16.04)
  *  A Connect 4 game board

### Installing

  1.  Clone or download the repository to your ROS `/src` folder (default location is `~/ros_ws/src`)
  2.  Connect to Baxter (or a simulator) using `baxter.sh` and enable him (`rosrun baxter_tools enable_robot.py -e`)
  3.  Start the vision components using `rosrun`:

  `$ rosrun connect_four connect_vision.py -l <hand>`
    *  The `-l` option specifies which limb will be used to pick up and place game pieces (`hand` can be either `left` or `right`)

  4.  Start the game using `rosrun`:

  `$ rosrun connect_four connect_game.py -l <hand> -c <colour>`
    *  The `-l` option specifies which limb will be used to pick up and place game pieces (should be the same limb specified in the previous command, `left` or `right`)
    *  The `-c` option specifies the colour game piece that Baxter is playing with (`colour` can be either `red` or `yellow`)

  5.  Follow the instructions on screen to play Connect 4 against Baxter!

## Authors

  *  Patrick Kenworthy (21148951)
  *  Luca Trimboli (21296303)
  *  Brett Leask (21504732)
  *  Alex Arnold (21304455)

## Acknowledgements

 *  Rethink Robotics, for their [source code](https://github.com/RethinkRobotics/baxter_sandbox) (on GitHub)
 *  Kai Li Lim and Prof. Thomas Braunl, for assisting us throughout the project
