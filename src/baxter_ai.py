#!/usr/bin/python

class BaxterAI(object):
    """Contains all the functions for determining the game state/next move"""

    def __init__(self, depth):
        """Set up local variables (alpha, beta, dictionary of weight scores)"""

        print "BaxterAI::__init__() called!"
        self.scores = {
            1: 0,
            2: 10,
            3: 100,
            4: 1000
        }

    def get_column(self, grid):
        """Returns the column for Baxter to place a game piece in."""

        print "BaxterAI::get_column() called!"
        return None

    def _alphabeta(self, grid, depth, alpha, beta, isMaxPlayer):
        """The Minimax game algorithm with alpha-beta pruning. Recursive (depth
        times). Returns the heuristic value of the given grid.
        """

        print "BaxterAI::_alphabeta() called!"
        return None

    def _get_sub_grids(self, grid, isMaxPlayer):
        """Returns a list of all possible grids/moves from the specified grid"""

        print "BaxterAI::_get_sub_grids() called!"
        return None

    def _get_heuristic_value(self, grid, isMaxPlayer):
        """Returns the heuristic value of the given grid"""

        score = 0
        score += self.scores.get(_horizontal_score(self, grid, row, col))
        score += self.scores.get(_vertical_score(self, grid, row, col))
        score += self.scores.get(_forwardslash_score(self, grid, row, col))
        score += self.scores.get(_backslash_score(self, grid, row, col))

        if not isMaxPlayer:
            return -1 * score
        return score

    def _horizontal_score(self, grid, row, col):
        """Returns the horizontal score of a piece placed at (row, col)"""

        print "BaxterAI::_horizontal_score() called!"
        return None

    def _vertical_score(self, grid, row, col):
        """Returns the vertical score of a piece placed at (row, col)"""

        print "BaxterAI::_vertical_score() called!"
        return None

    def _forwardslash_score(self, grid, row, col):
        """Returns the forward-slash (diagonal) score of a piece placed at
        (row, col)
        """

        print "BaxterAI::_fwdslash_score() called!"
        return None

    def _backslash_score(self, grid, row, col):
        """Returns the backslash (diagonal) score of a piece placed at
        (row, col)
        """

        print "BaxterAI::_backslash_score() called!"
        return None
