from turtle_brick.catcher import catch_dist


def test_negative_values():
    assert (catch_dist(-1, -1, 9.8, -10)) is False


def test_default_values():
    assert (catch_dist(20, 5, 2.8, 2.2)) is True
    assert (catch_dist(20, 10, 9.8, 0.22)) is False
