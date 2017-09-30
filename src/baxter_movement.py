#!/usr/bin/python

class BaxterMovement(object):
    """Contains all the functions to move the robots limbs"""

    def __init__(self, limb):
        """Sets up limbs and gripper"""

        print "BaxterMovement::__init__() called!"

    def set_poses(self):
        """Sets the poses for: camera limb to see board, picker limb to pick up
        playing piece, picker limb to see slots (just start and end slots or all
        board slots), picker limb to neutral position, both limbs to win/lose/
        draw positions (may actually be 'animations' - and hence functions).
        """

        print "BaxterMovement::set_poses() called!"

    def move(self, limb, pose):
        """Move selected limb to specified pose."""

        print "BaxterMovement::move() called!"
