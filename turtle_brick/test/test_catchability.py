import pytest
import unittest
from turtle_brick.catcher import catch_dist

def test_negative_values():
    assert (catch_dist(-1, -1, 9.8, -10)) == False

def test_default_values():
    assert (catch_dist(20, 10, 2.8, 2.2)) == True
    assert (catch_dist(20, 10, 9.8, 0.22)) == False