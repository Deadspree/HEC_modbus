--pick
x0, y0, z0, rx0, ry0, rz0 = GetActualTCPPose()
--MoveGripper(1,0,38,25,5000,0)
x,y,z,rx,ry,rz = 279.43073858796424,-491.2273944417306,11.208336172823907,180.0,-0.0,-91.16899322158068
RegisterVar("number", "x")
j1,j2,j3,j4,j5,j6 = GetInverseKin(0,x,y,z,rx,ry,rz,-1)
MoveJ(j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz,9,0,50,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0)
WaitMs(2000)
ActGripper(1,1)
MoveGripper(1,100,38,100,5000,0)
z = 70
j1,j2,j3,j4,j5,j6 = GetInverseKin(0,x,y,z,rx,ry,rz,-1)
MoveL(j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz,9,0,30, 180,100, -1, 0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0)
WaitMs(2000)
j1,j2,j3,j4,j5,j6 = GetInverseKin(0,x0,y0,z0,rx0,ry0,rz0,-1)
MoveJ(j1,j2,j3,j4,j5,j6,x0,y0,z0,rx0,ry0,rz0,9,0,50,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0)
MoveGripper(1,0,38,25,5000,0)