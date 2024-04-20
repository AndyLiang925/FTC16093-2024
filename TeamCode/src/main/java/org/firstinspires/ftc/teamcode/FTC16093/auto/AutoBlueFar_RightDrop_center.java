package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous (group = "AutoBlueFar")
@Disabled
public class AutoBlueFar_RightDrop_center extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;
        drop_side = RIGHT;
        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();

        //DistalBackDropDump();
        intakeDistal();
        sleep(wait_time);
        distalMoveToBackDrop();
        //DistalBackDropDump_farGrab();
        sleep(300);
        upper.setArmPosition(2100);
        sleep(1000);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        backDrop_move();

        upper.putOnBackDrop();
        sleep(200);
        setUpAuto();
        sleep(1500);
        //ecByCenter_far();
        //ec_far_putOnGround();
    }
}
