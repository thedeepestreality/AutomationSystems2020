# uvicorn server:app --reload
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
import pybullet
from concurrent.futures import ThreadPoolExecutor
from robot import NoSuchControlType, robot
import asyncio
from logger import Logger
from dtos import *

app = FastAPI()

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(robot.step_in_background())
    
@app.get("/robot/state")
async def get_robot_state():
    return robot.get_full_state()

@app.post("/robot/joints")
async def post_joints(new_state: JointsState):
    try:
        robot.set_control(new_state.scaling, new_state.ts, new_state.positions)
    except ValueError as err:
        return {"error": err.args[0]}
    return robot.get_full_state()

@app.post("/robot/joint_traj_lin")
async def joint_traj(traj: JointTrajLin):
    try:
        robot.set_traj_control_lin(traj.scaling, traj.traj)
    except ValueError as err:
        return {"error": err.args[0]}
    return robot.get_full_state()

@app.post("/robot/joint_traj_interp")
async def joint_traj_interp(traj: JointTrajInterp):
    try:
        robot.set_traj_control_interp(traj.interpolation, traj.traj)
    except ValueError as err:
        return {"error": err.args[0]}
    return robot.get_full_state()

@app.post("/robot/cart_traj")
async def cart_traj(traj: CartesianTraj):
    try:
        robot.set_cart_traj(traj.interpolation, traj.traj)
    except ValueError as err:
        return {"error": err.args[0]}
    return robot.get_full_state()

@app.post("/robot/cart_traj_screw")
async def cart_traj_screw(traj: CartesianTraj):
    try:
        robot.set_cart_traj_screw(traj.traj)
    except ValueError as err:
        return {"error": err.args[0]}
    return robot.get_full_state()

@app.post("/robot/compute_ik")
async def compute_ik(cart_state: CartesianState):
    return robot.get_inverse_kinematics(cart_state.pos, cart_state.orient)

@app.on_event("shutdown")
async def shutdown_event():
    pybullet.disconnect()
    Logger.close()


@app.exception_handler(NoSuchControlType)
async def unicorn_exception_handler(request: Request, exc: NoSuchControlType):
    return JSONResponse(
        status_code=400,
        content={"message": f"No such control type: {exc.args[0]}"},
    )
