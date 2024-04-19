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
        delay(wait_time);
        distalMoveToBackDrop();
        delay(500);
        upper.putOnBackDrop();

        upper.setArmPosition_slow(4100);
        delay(300);
        backDrop_move();
        upper.release_extra();
        delay(200);
        upper.setArmPosition(100);
        delay(1500);
        upper.setArmPosition(0);
        parking(2);
        //ec_far_putOnGround();

    }
}

