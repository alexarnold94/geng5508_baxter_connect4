#!/usr/bin/python

class BaxterConnectFour(object):
    """The class that controls Baxter and plays Connect 4."""

    def __init__(self, limb, baxter_colour, depth):
        """Setup local variables (alpha, beta, and the dictionary of score
        weights)
        """
        print 'BaxterConnectFour::__init__() called!'

    def play(self):
        """Call play() to start the game."""

        print "play() called!"

def main():
    """Parse arguments, create BaxterConnectFour instance, call .play() on that
    instance, handle playing a new game after completion(?)
    """
    print "main() called!"

if __name__ == "__main__":
    main()
