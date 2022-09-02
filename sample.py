from cgitb import reset
from email.mime import base
from pickle import FRAME
from sre_constants import BRANCH
from sre_parse import FLAGS
import pybullet as p
import pybullet_data as pd
import numpy as np
import time


class Robo_env():
    def __init__(self):
        self.RoboID = None
        self.sleep_t = 1. / 240  # decrease the value if it is too slow.
        self.maxVelocity = 0.5
        self.force = 18
        self.n_sim_steps = 100
        self.startPos = [0, 0, 0.5]
        self.startOri = [np.pi / 2, 0, 0]
        self.initial_action = []
        self.jointIds = []
        self.reset()
        self.friction = 0.5

    
    
    def getInversePos(self,EndEffIndex,jointRange,pos_desired,ori_desired=[]): ##endeffindex is the joint that is interacting with the env
        joint_info = []
        
        for i in jointRange:
            joint_info.append(p.getJointInfo(self.RoboID,i))
            
        pos = [pos_desired[0],pos_desired[1],pos_desired[2]]
        
        if ori_desired != []: ## orientation direction
              ori = p.getQuaternionFromEuler([ori_desired[0],ori_desired[1] , ori_desired[2]])
              jointPos = p.calculateInverseKinematics(self.RoboID,EndEffIndex,pos,ori)
        
        else:
             jointPos = p.calculateInverseKinematics(self.RoboID,EndEffIndex,pos) 

        #print(jointPos[0:3])
        
        return jointPos
          
        
    def MoveFLJoint(self,pos):  ##move the 
      FLjointPos = self.getInversePos(EndEffIndex=2,jointRange=[0,1,2],pos_desired=pos)[0:3] #Get the moving position for each joint
      
      for i in range(0,3):
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[i],
                                    targetPosition=FLjointPos[i],
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
      # for _ in range(self.n_sim_steps):   # start the simulation with the number of the step
      #       p.stepSimulation()
      #       time.sleep(1. / 100)
            
    def MoveFRJoint(self,pos):  
      FRjointPos = self.getInversePos(EndEffIndex=5,jointRange=[3,4,5],pos_desired=pos)[3:6]     
      for i in range(3,6):
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[i],
                                    targetPosition=FRjointPos[i-3],
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
      # for _ in range(self.n_sim_steps):   # start the simulation with the number of the step
      #       p.stepSimulation()
      #       time.sleep(1. / 100)
      
    def MoveBLJoint(self,pos):  
      BLjointPos = self.getInversePos(EndEffIndex=8,jointRange=[6,7,8],pos_desired=pos)[6:9]
      for i in range(6,9):
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[i],
                                    targetPosition=BLjointPos[i-6],
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
      # for _ in range(self.n_sim_steps):   # start the simulation with the number of the step
      #       p.stepSimulation()
      #       time.sleep(1. / 100)
     
     
    def MoveBRJoint(self,pos): 
      BRjointPos = self.getInversePos(EndEffIndex=11,jointRange=[9,10,11],pos_desired=pos)[9:12]              
      for i in range(9,12):
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[i],
                                    targetPosition=BRjointPos[i-9],
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
                  
      # for _ in range(self.n_sim_steps):   # start the simulation with the number of the step
      #       p.stepSimulation()
      #       time.sleep(1. / 100)     
  
    
    def getJointPosition(self):   #Get the joint global position
          
        basePos = []
        basePos = p.getBasePositionAndOrientation(self.RoboID)[0]
        
        baseXPos = basePos[0] #base pos
        baseYPos = basePos[1]
        baseZPos = basePos[2]
        
        jointInfo=[]
        for i in range (12):
              jointInfo.append((p.getJointInfo(self.RoboID,i))[14])
        
        #print(jointInfo)
        
        FLEndRelativePos = jointInfo[2]  ## relative pos to the base
        FREndRelativePos = jointInfo[5]
        BLEndRelativePos = jointInfo[8]
        BREndRelativePos = jointInfo[11]
        
        
        # FLEndPos = [baseXPos+FLEndRelativePos[0], baseYPos+FLEndRelativePos[1],baseZPos+FLEndRelativePos[2]]  #abosolute pos in the globe
        # FREndPos = [baseXPos+FREndRelativePos[0], baseYPos+FREndRelativePos[1],baseZPos+FREndRelativePos[2]]
        # BLEndPos = [baseXPos+BLEndRelativePos[0], baseYPos+BLEndRelativePos[1],baseZPos+BLEndRelativePos[2]]
        # BREndPos = [baseXPos+BREndRelativePos[0], baseYPos+BREndRelativePos[1],baseZPos+BREndRelativePos[2]]
        
        FLEndPos = [baseXPos+FLEndRelativePos[0], baseYPos+FLEndRelativePos[1],baseZPos+FLEndRelativePos[2]]  #abosolute pos in the globe
        FREndPos = [baseXPos+FREndRelativePos[0], baseYPos+FREndRelativePos[1],baseZPos+FREndRelativePos[2]]
        BLEndPos = [baseXPos+BLEndRelativePos[0], baseYPos+BLEndRelativePos[1],baseZPos+BLEndRelativePos[2]]
        BREndPos = [baseXPos+BREndRelativePos[0], baseYPos+BREndRelativePos[1],baseZPos+BREndRelativePos[2]]
        
        return FLEndPos,FREndPos,BLEndPos,BREndPos



    def step(self, action):
       
            
        jointState = []
        for i in range(12):
            jointState.append(p.getJointState(self.RoboID,i)[0])
        
            
        for j in range(0,12):
            
            targetPos = float(action[j])
            targetPos = jointState[j]+0.5
            #print(targetPos)
            targetPos = self.jointDirections[j] * targetPos + self.jointOffsets[j]
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[j],
                                    targetPosition=targetPos,
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
            
            #print(jointState)
        for _ in range(self.n_sim_steps):   # start the simulation with the number of the step
            p.stepSimulation()
            time.sleep(1. / 100)  
       
         


    def reset(self):

        p.resetSimulation()
        p.setGravity(0, 0, -10)
        planeId = p.loadURDF("plane.urdf")  # URDF Id = 0
        self.RoboID = p.loadURDF("laikago/laikago.urdf", self.startPos,
                                 p.getQuaternionFromEuler(self.startOri))  # URDF Id = 1

        self.jointOffsets = []
        self.jointDirections = [-1, -1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1]

        for i in range(4):
            self.jointOffsets.append(0)
            self.jointOffsets.append(0.1)
            self.jointOffsets.append(-0.1)
        
        for i in range(12):
            p.changeDynamics(self.RoboID,i,lateralFriction = 0.5,rollingFriction=0.5)


        for j in range(p.getNumJoints(self.RoboID)):
            p.changeDynamics(self.RoboID, j, linearDamping=0.5, angularDamping=0)
            info = p.getJointInfo(self.RoboID, j)
            #print(info)
            jointName = info[1]
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                self.jointIds.append(j)
        
        


        # with open("data1.txt", "r") as filestream:
        #     for line in filestream:
        #         currentline = line.split(",")
        #         joints = currentline[2:14]
        #         self.step(joints)
        #         self.initial_action = joints
                #print(self.initial_action)
                


if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pd.getDataPath())  # optionally
    env1 = Robo_env()
    #print(env1.initial_action)
    c = 0
    
    
   # env1.MoveForward()
    #env1.getInversePos(2,range(0,12),[2,1,1])
    while 1:
  
        currentPos = env1.getJointPosition()
        #print(currentPos)
        
        FLPos = currentPos[0]  ##to get the cureent joint pos
        FRPos = currentPos[1]
        BLPos = currentPos[2]
        BRPos = currentPos[3]
        
        Increment = [0.0000000000001,0,0]
        
        ### list addition
        a_array =np.array(Increment) #increment of every step
        
       
        b_array = np.array(FLPos) 
        c_array = np.array(FRPos)
        d_array = np.array(BLPos)
        e_array = np.array(BRPos)
        
      
      
        FL = list(a_array+b_array)
        FR = list(a_array+c_array)
        BL = list(a_array+d_array)
        BR = list(a_array+e_array)
        

        env1.MoveFLJoint([BL[0],BL[1],-0.5])
        p.stepSimulation()
        env1.MoveBLJoint([BR[0],BR[1],-0.5])
        p.stepSimulation()
        env1.MoveFRJoint([FL[0],FL[1],-0.5])
        p.stepSimulation()
        env1.MoveBRJoint([FR[0],FR[1],-0.5])
        p.stepSimulation()
        c += 1
        time.sleep(0.005)
        

        
       
