package org.firstinspires.ftc.teamcode.FTC16093.auto.extraPath;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;

@Autonomous
public class splineIntakeTest extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        spikeMarkDump();
        extraIntakeSpline();
        intake2();
        setUpAuto();
    }

}

