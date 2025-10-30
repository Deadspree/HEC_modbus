tool = 1 --tool number
user = 0 --Object number
vel = 30
acc = 70
ovl = 50
blendT = 50
config = 1
desc_pos1 = {0,0,0,0,0,0}
error_log = ModbusMasterReadAO(Modbus_0,error_log,1)
while (1) do
    WaitMs(800)
    reg_add = ModbusMasterReadAO(Modbus_0,register_address,1)
    if reg_add == 1 then
        x = ModbusMasterReadAO(Modbus_0,x_val,1)
        y = ModbusMasterReadAO(Modbus_0,y_val,1)
        z = ModbusMasterReadAO(Modbus_0,z_val,1)
        rx = ModbusMasterReadAO(Modbus_0,rx_val,1)
        ry = ModbusMasterReadAO(Modbus_0,ry_val,1)
        rz = ModbusMasterReadAO(Modbus_0,rx_val,1)
        desc_pos1 = {x, y, z, rx, ry, rz}
        ModbusMasterWriteAO(Modbus_0,register_address,1,{0})
    end
    WaitMs(800)
    motion_trigger = ModbusMasterReadAO(Modbus_0,trigger_move,1)
    if motion_trigger == 1 and GetRobotMotionDone() == 1 then
        MoveCart(desc_pos1, tool, user, vel, acc, ovl, blendT, config)
        WaitMs(5000)
        ModbusMasterWriteAO(Modbus_0,trigger_move,1,{0})
        ModbusMasterWriteAO(Modbus_0,x_val,1,{0})
        ModbusMasterWriteAO(Modbus_0,y_val,1,{0})
        ModbusMasterWriteAO(Modbus_0,z_val,1,{0})
        ModbusMasterWriteAO(Modbus_0,rx_val,1,{0})
        ModbusMasterWriteAO(Modbus_0,ry_val,1,{0})
        ModbusMasterWriteAO(Modbus_0,rz_val,1,{0})
    end
    WaitMs(800)
end
