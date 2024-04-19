package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedFar")
public class AutoRedFar_RightDrop_centerIntake_gate extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = RED;
        drop_side = RIGHT;
        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();

        intakeDistal();
        sleep(wait_time);
        distalMoveToBackDrop();
        sleep(300);
        upper.setArmPosition(4200);
        sleep(1000);
        upper.putOnBackDrop();

        upper.setArmPosition(4100);
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

