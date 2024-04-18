package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoBlueFar")
public class AutoBlueFar_RightDrop_sideIntake_sideBack extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;
        drop_side = RIGHT;

        initHardware();
        spikeMarkDump();
        upper.putOnSpikeMark();

        //DistalBackDropDump();
        distal_intakeEdge_edgeBack();
        upper.setArmPosition(2100);
        sleep(1000);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        backDrop_move();
        upper.putOnBackDrop();
        sleep(200);
        setUpAuto();
        sleep(1500);
    }
}