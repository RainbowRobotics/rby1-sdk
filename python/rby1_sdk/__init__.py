from __future__ import annotations

from ._bindings import *
from ._bindings import __version__
# from ._robot_command import *

from typing import overload

@overload
def create_robot(address, model_name: str) -> Robot_A: ...
@overload
def create_robot(address, model_name: str) -> Robot_M: ...
@overload
def create_robot(address, model_name: str) -> Robot_T5: ...

def create_robot(address, model_name: str):
  return _bindings._create_robot(address, model_name)