package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous (group = "AutoRedFar")
@Disabled
public class AutoRedFar_LeftDrop_centerIntake_gate extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = RED;
        drop_side = LEFT;
        initHardware();

        moveToSpikeMark();
        upper.putOnSpikeMark();

        intakeDistal();
        sleep(wait_time);
        distalMoveToBackDrop();

        sleep(500);
        upper.putOnBackDrop();
        upper.setArmPosition(2000);
        sleep(300);

        backDrop_move();

        upper.release_extra();
        sleep(200);
        setUpAuto();
        sleep(1500);
        //parking(2);
        //ec_far_putOnGround();

    }
}

