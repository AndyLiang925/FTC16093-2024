package org.firstinspires.ftc.teamcode.FTC16093.auto.extraPath;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;

@Autonomous
public class AutoTestLinearSplineIntake extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {

        startSide = PROXIMAL;
        side_color = BLUE;
        initHardware();

        spikeMarkDump();
        putOnSpikeMark();

        backDropDump();
        putOnBackDrop();
        sleep(500);

        setUpAuto();
        sleep(1000);

        extraIntakeLinearBySpline();
        putOnBackDrop();
        sleep(500);
        setUpAuto();
        sleep(1500);
    }

}
