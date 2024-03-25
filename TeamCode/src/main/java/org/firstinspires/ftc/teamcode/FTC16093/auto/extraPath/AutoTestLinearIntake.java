package org.firstinspires.ftc.teamcode.FTC16093.auto.extraPath;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;

@Autonomous
public class AutoTestLinearIntake extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {

        startSide = PROXIMAL;
        side_color = BLUE;
        initHardware();

        spikeMarkDump();

        putOnSpikeMark();
        backDropDump();
        putOnBackDrop();
        sleep(1000);
        setUpAuto();


        extraIntakeLinearPath();

        putOnBackDrop();
        sleep(1500);
        setUpAuto();
        sleep(1500);
    }
}
