#!/usr/bin/python

class BaxterVision(object):
    """Contains all the functions for image processing. Used by the AI to
    determine what the current game state is.
    """

    def __init__(self, limb):
        """Sets up camera"""

        print "BaxterVision::__init__() called!"

    def get_grid(self):
        """High-level function that BaxterAI can use to determine game state"""

        print "BaxterVision::get_grid() called!"
        return None

    def _on_camera(self):
        """Callback function for the camera subscriber. Converts the image to
        OpenCV format, then extracts and saves the game board sub-image.
        """

        print "BaxterVision::_on_camera() called!"

    def _get_grid(self):
        """Get Connect 4 board layout from camera sub-image"""

        print "BaxterVision::_get_grid() called!"
        return None

    def _filter_image(self, lower, upper):
        """Converts the sub-image to HSV colourspace, then binarises the image
        along the H-dimension between the upper and lower values.
        """

        print "BaxterVision::_filter_image() called!"
        return None

    def _process_colours(self):
        """Update grid with red and yellow positions."""

        print "BaxterVision::_process_colours() called!"
        return None
