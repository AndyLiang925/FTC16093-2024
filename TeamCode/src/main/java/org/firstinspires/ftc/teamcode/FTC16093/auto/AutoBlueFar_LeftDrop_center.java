package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoBlueFar")
public class AutoBlueFar_LeftDrop_center extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;
        drop_side = LEFT;
        initHardware();

        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();

        // 夹1个白片
        intakeDistal();
        sleep(wait_time);

        // 到背板放片
        distalMoveToBackDrop();
        upper.putOnBackDrop();

        upper.setArmPosition(4050);
        //backDrop_move();
        moveToDropMiddle();
        upper.release_extra();
        delay(200);
        upper.setArmPosition(0);
        upper.setSlide(0,0.85);
//        intakeGate();
//        upper.drop_upward();
//        upper.setArmPosition(0);
        parking(2);
        //ecByCenter_far();
        //ec_far_putOnGround();
    }
}

