# uvicorn server:app --reload
from fastapi import FastAPI
import pybullet
from typing import List
from concurrent.futures import ThreadPoolExecutor
from robot import step_in_background, move_to_position, get_joint_position
import asyncio
from pydantic import BaseModel

app = FastAPI()

class JointsState(BaseModel):
    ids: List[int]
    positions: List[float]

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(step_in_background())
    
@app.get("/robot/joints")
async def get_joints():
    return get_joint_position()

@app.post("/robot/joints")
async def post_joints(new_state: JointsState):
    move_to_position(new_state.ids, new_state.positions)
    return get_joint_position()

@app.on_event("shutdown")
async def shutdown_event():
    pybullet.disconnect()
