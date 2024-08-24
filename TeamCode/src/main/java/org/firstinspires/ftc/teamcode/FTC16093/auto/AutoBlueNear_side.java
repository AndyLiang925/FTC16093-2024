package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoBlueNear")
@Config
public class AutoBlueNear_side extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = BLUE;
        // 初始化
        initHardware();
        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();
        // 放黄片
        moveToDropYellow_Near();
        upper.dropYellow();

        upper.setArmPosition(0);

        // Side夹2个白片
        intakeSide();
        delay(100);
        upper.wristtoOrigin();
        side_moveToDropUpward();
        upper.drop_upward();
        upper.dropToOrigin();

        // Side夹2个白片
        intakeSide();
        delay(100);
        upper.wristtoOrigin();
        side_moveToDropUpward();
        upper.drop_upward();
        // 回到初始状态
        upper.toOrigin();

        parking(1);
    }
}
