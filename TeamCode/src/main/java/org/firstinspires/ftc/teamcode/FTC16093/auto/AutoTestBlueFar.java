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
        sleep(300);
        putOnBackDrop();
        sleep(1000);
        setUpAuto();
    }
}

