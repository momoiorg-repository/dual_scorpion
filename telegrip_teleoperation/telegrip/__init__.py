"""TeleGrip - dual_scorpion teleoperation system, with lazy optional backends."""

from importlib import import_module

__version__ = "0.2.0"
_EXPORTS = {
    "RobotInterface": (".core.robot_interface", "RobotInterface"),
    "Visualizer": (".core.visualizer", "PyBulletVisualizer"),
    "ControlLoop": (".control_loop", "ControlLoop"),
    "TelegripConfig": (".config", "TelegripConfig"),
    "load_config": (".config", "load_config"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    module, attribute = _EXPORTS[name]
    value = getattr(import_module(module, __name__), attribute)
    globals()[name] = value
    return value
