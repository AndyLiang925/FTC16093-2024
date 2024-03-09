package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestBlueNear extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = PROXIMAL;
        side_color = BLUE;
        initHardware();

        setKickProp();
        spikeMarkDump();
        putOnSpikeMark();
        backDropDump();

        sleep(1000);
        putOnBackDrop();
        setUpAuto();
        extraCredit();
        putOnBackDrop();
        setUpAuto();
    }

}

