
tool = 0 --tool number
user = 0 --Object number
vel = 70
acc = 70
ovl = 50
blendT = 50
config = 1
desc_pos1 = {0,0,0,0,0,0}
joint_pos_ref = {0,0,0,0,0,0}
possible = 1
while (1) do
    WaitMs(800)
    if GetRobotMotionDone() == 1 then
        x = ModbusMasterReadAO(Modbus_0,x_val,1)
        y = ModbusMasterReadAO(Modbus_0,y_val,1)
        z = ModbusMasterReadAO(Modbus_0,z_val,1)
        rx = ModbusMasterReadAO(Modbus_0,rx_val,1)
        ry = ModbusMasterReadAO(Modbus_0,ry_val,1)
        rz = ModbusMasterReadAO(Modbus_0,rz_val,1)
        desc_pos1 = {x,y,z, rx, ry, rz}
    end
    WaitMs(2000)
    j1,j2,j3,j4,j5,j6 = GetActualJointPosDegree()
    joint_pos_ref = {j1,j2,j3,j4,j5,j6}
    motion_trigger = ModbusMasterReadAO(Modbus_0,trigger_move,1)
    if motion_trigger == 1 and GetRobotMotionDone() == 1 then
        possible = GetInverseKinHasSolution(0,desc_pos1,joint_pos_ref)
        if possible == 1 then
            MoveCart(desc_pos1, tool, user, vel, acc, ovl, blendT, config)
            WaitMs(1000)
            while (1) do
                if GetRobotMotionDone() == 1 then
                    ModbusMasterWriteAO(Modbus_0,move_done,1,{1})
                    break
                end
            end
            WaitMs(4000)
            ModbusMasterWriteAO(Modbus_0,trigger_move,1,{0})
            ModbusMasterWriteAO(Modbus_0,move_done,1,{0})
        else
            ModbusMasterWriteAO(Modbus_0,move_done,1,{1})
            WaitMs(4000)
            ModbusMasterWriteAO(Modbus_0,trigger_move,1,{0})
            ModbusMasterWriteAO(Modbus_0,move_done,1,{0})
        end
    end
    WaitMs(800)
end