from par_core import Deadman


def test_disarmed_before_first_heartbeat() -> None:
    d = Deadman(timeout_s=0.3)
    assert d.is_armed(now_s=0.0) is False


def test_armed_within_timeout() -> None:
    d = Deadman(timeout_s=0.3)
    d.heartbeat(now_s=0.0)
    assert d.is_armed(now_s=0.1) is True


def test_disarmed_after_timeout() -> None:
    d = Deadman(timeout_s=0.3)
    d.heartbeat(now_s=0.0)
    assert d.is_armed(now_s=1.0) is False
