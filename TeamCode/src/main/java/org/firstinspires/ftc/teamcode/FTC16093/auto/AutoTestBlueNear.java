package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestBlueNear extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = PROXIMAL;
        side_color = BLUE;
        initHardware();

        //setKickProp();
        spikeMarkDump();
        putOnSpikeMark();
        backDropDump();


        putOnBackDrop_grab2();
        setUpAuto();

        extraCredit();

        putOnBackDrop_grab2();
        sleep(300);
        setUpAuto();
        sleep(1000);
    }

}

