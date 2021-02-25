#!/usr/bin/env python3
from fastapi import FastAPI
import pybullet
from typing import List
from concurrent.futures import ThreadPoolExecutor
from robot import step_in_background, move_to_position, get_state, get_ik
import asyncio
from pydantic import BaseModel
from logger import Logger

app = FastAPI()

class JointsState(BaseModel):
    ids: List[int]
    positions: List[float]
    ts: float
    scaling: str


class CartesianState(BaseModel):
    pos: List[float]
    orient: List[float]

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(step_in_background())
    
@app.get("/robot/state")
async def get_robot_state():
    return get_state()

@app.post("/robot/joints")
async def post_joints(new_state: JointsState):
    try:
        move_to_position(new_state.ids, new_state.positions, new_state.ts, new_state.scaling)
    except ValueError as err:
        return {"error": err.args[0]}
    return get_state()

@app.post("/robot/compute_ik")
async def compute_ik(cart_state: CartesianState):
    return get_ik(cart_state.pos, cart_state.orient)

@app.on_event("shutdown")
async def shutdown_event():
    pybullet.disconnect()
    Logger.close()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=5000, log_level="info")