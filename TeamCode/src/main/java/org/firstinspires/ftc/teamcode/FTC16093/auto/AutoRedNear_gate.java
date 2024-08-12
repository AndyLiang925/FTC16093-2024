package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoRedNear")
@Config
public class AutoRedNear_gate extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;
        // 初始化
        initHardware();
        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();
        // 放黄片
        moveToBackDrop();
        delay(200);
        //upper.putOnBackDrop();
        upper.releaseYellow(side_color);
        delay(200);
        upper.setArmPosition(0);
        // 夹2个白片
        intakeGate_simpleMove();
        // 放2个白片
        moveToDropUpward();
        upper.drop_upward();
        // 放下大臂，回收滑轨
        upper.dropToOrigin();

        // 重复
        intakeGate_simpleMove();
        moveToDropUpward();
        upper.drop_upward();
        upper.dropToOrigin();

        intakeGate_simpleMove();
        moveToDropUpward();
        upper.drop_upward();
        // 回到初始状态
        upper.toOrigin();
    }
}
