package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedFar")
public class AutoRedFar_RightDrop_sideIntake_sideBack extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = RED;
        drop_side = RIGHT;
        initHardware();

        spikeMarkDump();
        upper.putOnSpikeMark();
        distal_intakeEdge_edgeBack();
        sleep(300);
        upper.setArmPosition(2077);
        sleep(1000);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        sleep(300);

        backDrop_move();

        upper.putOnBackDrop();
        sleep(200);
        setUpAuto();
        sleep(1500);
        //parking(2);
        //ec_far_putOnGround();
    }
}

