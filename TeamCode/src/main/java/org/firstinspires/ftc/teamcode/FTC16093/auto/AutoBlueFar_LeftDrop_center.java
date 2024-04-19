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
        moveToSpikeMark();

        upper.putOnSpikeMark();

        //DistalBackDropDump();
        intakeDistal();
        sleep(wait_time);
        distalMoveToBackDrop();

        //DistalBackDropDump_farGrab();
        sleep(300);
        upper.setArmPosition_slow(4200); //2077
        sleep(1000);
        upper.putOnBackDrop();
        upper.setArmPosition(4100);
        backDrop_move();
        upper.release_extra();
        sleep(200);
        setUpAuto();
        sleep(1500);
        parking(2);
        //ecByCenter_far();
        //ec_far_putOnGround();
    }
}

