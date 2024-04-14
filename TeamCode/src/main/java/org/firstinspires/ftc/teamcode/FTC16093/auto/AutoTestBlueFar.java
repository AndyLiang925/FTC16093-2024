package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestBlueFar extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;

        initHardware();
        spikeMarkDump();
        putOnSpikeMark();

        DistalBackDropDump();
        //DistalBackDropDump_farGrab();
        sleep(300);
        upper.setArmPosition(2100);
        sleep(500);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        backDrop_move();
        
        putOnBackDrop_grab1();
        sleep(200);
        setUpAuto();
        //ecByCenter_far();
        ec_far_putOnGround();
    }
}

