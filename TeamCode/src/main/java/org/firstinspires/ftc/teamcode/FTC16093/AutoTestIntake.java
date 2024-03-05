package org.firstinspires.ftc.teamcode.FTC16093;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTestIntake extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();

        setUpAuto();
        intake2();
        setUpAuto();
    }

}
