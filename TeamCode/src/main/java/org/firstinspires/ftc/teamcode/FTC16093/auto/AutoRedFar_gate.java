package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedFar")
public class AutoRedFar_gate extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = DISTAL;
        side_color = RED;

        // 初始化
        initHardware();

        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();
        delay(wait_time);

        // 夹1个白片
        intakeDistal();

        // 到背板放黄片
        distalMoveToBackDrop();
        upper.putOnBackDrop();

        // 移到板侧放1个白片
        upper.setArmPosition(4100);
        moveToDropWhite();
        upper.release_extra();
        upper.dropToOrigin();

        // 夹2个白片
        intakeGate_simpleMove();
        // 放2个白片
        moveToDropUpward();
        upper.drop_upward();
        // 放下大臂，收回滑轨,回到初始位置
        upper.toOrigin();

    }
}

