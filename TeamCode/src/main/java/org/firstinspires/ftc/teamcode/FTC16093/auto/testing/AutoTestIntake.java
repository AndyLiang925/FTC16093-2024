package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC16093.AutoMaster;

@Autonomous
public class AutoTestIntake extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();

        intake2();
        setUpAuto();
    }

}
