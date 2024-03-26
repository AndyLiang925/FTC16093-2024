package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;

@Autonomous
public class AutoTestIntake extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        initHardware();
        //resetUpper();
        //intake2();
        intake2();
        //setUpAuto();
        sleep(300);
        //intake_throw();
        //putOnBackDrop();

        setUpAuto();
        //sleep(1000);

    }

}
