# import pybullet as p
# from time import sleep
#
# physicsClient = p.connect(p.GUI)
#
# p.setGravity(0, 0, -10)
# planeId = p.loadURDF("plane.urdf")
# cubeStartPos = [0, 0, 1]
# cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# boxId = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#
# useRealTimeSimulation = 0
#
# if (useRealTimeSimulation):
#   p.setRealTimeSimulation(1)
#
# while 1:
#   if (useRealTimeSimulation):
#     p.setGravity(0, 0, -10)
#     sleep(0.01)  # Time in seconds.
#   else:
#     p.stepSimulation()
# import pybullet as p
# import pybullet_data as pd
# import math
# import time
#
# # cameraTargetPosition=[0.55,-0.35,0.2]
# p.connect(p.GUI)
# p.setGravity(0, 0, -10)
# # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
# p.setAdditionalSearchPath(pd.getDataPath())
# p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0, \
#                              cameraPitch=-40, cameraTargetPosition=[0.5, -0.9, 0.5])
#
# pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
# tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])
# trayUid = p.loadURDF("tray/traybox.urdf", basePosition=[0.65, 0, 0])
#
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.1])
# # object1Uid=p.loadURDF("random_urdfs/001/001.urdf",basePosition=[0.7,0.1,0.7])
# # object2Uid=p.loadURDF("random_urdfs/002/002.urdf",basePosition=[0.6,-0.2,0.7])
# # object3Uid=p.loadURDF("random_urdfs/003/003.urdf",basePosition=[0.8,0.1,0.7])
# # object4Uid=p.loadURDF("random_urdfs/004/004.urdf",basePosition=[0.7,-0.1,0.7])
# # object5Uid=p.loadURDF("random_urdfs/005/005.urdf",basePosition=[0.6,0.1,0.3])
# # object6Uid=p.loadURDF("random_urdfs/006/006.urdf",basePosition=[0.8,-0.2,0.4])
# # object7Uid=p.loadURDF("random_urdfs/007/007.urdf",basePosition=[0.6,0.2,0.4])
# # object8Uid=p.loadURDF("random_urdfs/008/008.urdf",basePosition=[0.7,-0.1,0.4])
# # object9Uid=p.loadURDF("random_urdfs/009/009.urdf",basePosition=[0.6,0.1,0.4])
#
# state_durations = [1, 1, 1, 1]
# control_dt = 1. / 240.
# p.setTimeStep = control_dt
# state_t = 0.
# current_state = 0
#
# while True:
#
#   state_t += control_dt
#   # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
#
#   if current_state == 0:
#     p.setJointMotorControl2(pandaUid, 0, p.POSITION_CONTROL, 0)
#     p.setJointMotorControl2(pandaUid, 1, p.POSITION_CONTROL, math.pi / 4.)
#     p.setJointMotorControl2(pandaUid, 2, p.POSITION_CONTROL, 0)
#     p.setJointMotorControl2(pandaUid, 3, p.POSITION_CONTROL, -math.pi / 2.)
#     p.setJointMotorControl2(pandaUid, 4, p.POSITION_CONTROL, 0)
#     p.setJointMotorControl2(pandaUid, 5, p.POSITION_CONTROL, 3 * math.pi / 4)
#     p.setJointMotorControl2(pandaUid, 6, p.POSITION_CONTROL, -math.pi / 4.)
#     p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
#     p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)
#
#   if current_state == 1:
#     p.setJointMotorControl2(pandaUid, 1, p.POSITION_CONTROL, math.pi / 4. + .15)
#     p.setJointMotorControl2(pandaUid, 3, p.POSITION_CONTROL, -math.pi / 2. + .15)
#
#   if current_state == 2:
#     p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, force=200)
#     p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, force=200)
#
#   if current_state == 3:
#     p.setJointMotorControl2(pandaUid, 1, p.POSITION_CONTROL, math.pi / 4. - 1)
#     p.setJointMotorControl2(pandaUid, 3, p.POSITION_CONTROL, -math.pi / 2. - 1)
#
#   if state_t > state_durations[current_state]:
#     current_state += 1
#     if current_state >= len(state_durations):
#       current_state = 0
#     state_t = 0
#
#   p.stepSimulation()

import os
import pybullet as p
import pybullet_data
import time

# 连接物理引擎
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

# 重力
p.setGravity(0, 0, -10)

# 实时仿真
useRealTimeSim = 1
p.setRealTimeSimulation(useRealTimeSim)

# 加载地面
# p.loadURDF("plane.urdf")
p.loadSDF("stadium.sdf")

# 加载小测
car = p.loadURDF("racecar/racecar.urdf")

inactive_wheels = [3, 5, 7]
wheels = [2]

for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

# 转向轮
steering = [4, 6]

# 自定义参数滑块，分别为速度，转向，驱动力
targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)

# 开始仿真
while 1:
  # 读取速度，转向角度，驱动力参数
  maxForce = p.readUserDebugParameter(maxForceSlider)
  targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
  steeringAngle = p.readUserDebugParameter(steeringSlider)
  # print(targetVelocity)

  # 根据上面读取到的值对关机进行设置
  for wheel in wheels:
    p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)

  for steer in steering:
    p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

  if useRealTimeSim == 0:
    p.stepSimulation()
