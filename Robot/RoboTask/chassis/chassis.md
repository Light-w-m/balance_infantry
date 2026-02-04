


    关节电机id设置--下方为接收id，发送id为 接收id+0x10
    在坐标系中，前方为phi1
    T2 为phi4角，T1为phi1角

        0x01         /\         0x03
    joint_motor[2]   ||     joint_motor[0]
        0x02         ||         0x04
    joint_motor[3]   ||     joint_motor[1]
        左           前          右

    完成进度:
        已完成：
            未测试：
                小板凳，离地检测，基础转向(小陀螺)
            已测试：
                
        未完成：
            倒地自起，跳跃，LQR串联MPC，单边桥，tof测距(自动检测跳跃)...