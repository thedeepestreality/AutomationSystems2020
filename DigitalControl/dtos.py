from pydantic import BaseModel
from typing import List

class JointsState(BaseModel):
    positions: List[float]
    ts: float
    scaling: str

class JointTarget(BaseModel):
    pos: List[float]
    ts: float

class JointTrajLin(BaseModel):
    traj: List[JointTarget]
    scaling: str

class JointTrajInterp(BaseModel):
    traj: List[JointTarget]
    interpolation: str

class CartesianState(BaseModel):
    pos: List[float]
    orient: List[float]

class CartesianTarget(CartesianState):
    ts: float

class CartesianTraj(BaseModel):
    traj: List[CartesianTarget]
    interpolation: str