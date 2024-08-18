package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedNear")
@Config
public class AutoRedNear_side extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;
        // 初始化
        initHardware();

        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();

        //放黄片
        moveToDropYellow_Near();
        upper.dropYellow();
        upper.dropToOrigin();

        // Side夹2个白片
        intakeSide();
        side_moveToDropUpward();
        upper.drop_upward();
        // 回到初始状态
        upper.toOrigin();

        parking(2);
    }
}
