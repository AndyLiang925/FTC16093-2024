package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoBlueFar")
public class AutoBlueFar_gate extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;

        // 初始化
        initHardware();

        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();

        // 夹1个白片
        intakeDistal();
        delay(10000);
        upper.grabWhite1_Finish();
        delay(wait_time);

        // 到背板放黄片
        distal_moveToDropYellow();
        upper.dropYellow();

        // 放1个白片
        upper.setArmPosition(4050);
        moveToDropWhite();
        upper.release_extra();

        delay(200);
        upper.setArmPosition(0);
        upper.setSlide(0,0.85);

        // 停靠到中间
        parking(2);
    }
}

