"""Unittest discovery bridge for the focused pytest-style checks."""

import unittest

from . import test_endpoint_safety
from . import test_exploration_policy
from . import test_frontier_search
from . import test_node_state


def load_tests(_loader, _tests, _pattern):
    suite = unittest.TestSuite()
    for module in (
        test_endpoint_safety,
        test_exploration_policy,
        test_frontier_search,
        test_node_state,
    ):
        for name in sorted(dir(module)):
            function = getattr(module, name)
            if name.startswith('test_') and callable(function):
                suite.addTest(unittest.FunctionTestCase(function))
    return suite
