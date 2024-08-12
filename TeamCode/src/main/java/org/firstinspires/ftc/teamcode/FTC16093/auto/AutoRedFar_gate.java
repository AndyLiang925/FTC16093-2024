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
        upper.setArmPosition_slow(4100);
        moveToDropWhite();
        upper.release_extra();
        delay(200);

        // 放下大臂，收回滑轨
        upper.setArmPosition(0);
        upper.setSlide(0,0.8);
        delay(1500);
    }
}

