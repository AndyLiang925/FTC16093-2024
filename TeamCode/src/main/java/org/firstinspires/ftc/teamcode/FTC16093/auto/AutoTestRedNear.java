package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTestRedNear extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = PROXIMAL;
        side_color = RED;
        initHardware();
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

