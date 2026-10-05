"""Lazy imports keep numerical/source tooling independent from robot drivers."""

from importlib import import_module

_EXPORTS = {
    "RobotInterface": (".robot_interface", "RobotInterface"),
    "IKSolver": (".kinematics", "IKSolver"),
    "ForwardKinematics": (".kinematics", "ForwardKinematics"),
    "PyBulletVisualizer": (".visualizer", "PyBulletVisualizer"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    module, attribute = _EXPORTS[name]
    value = getattr(import_module(module, __name__), attribute)
    globals()[name] = value
    return value
