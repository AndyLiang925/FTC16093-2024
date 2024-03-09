package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;

@Autonomous
public class AutoTestIntake extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();

        intake2();
        setUpAuto();
    }

}
