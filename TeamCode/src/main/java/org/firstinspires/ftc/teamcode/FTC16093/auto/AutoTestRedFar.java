package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTestRedFar extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = RED;
        initHardware();

        spikeMarkDump();
        putOnSpikeMark();

        DistalBackDropDump();
        sleep(300);
        upper.setArmPosition(2077);
        sleep(500);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        sleep(300);

        backDrop_move();

        putOnBackDrop_grab1();
        sleep(200);
        setUpAuto();
        //sleep(1500);
        //parking(2);
        ec_far_putOnGround();

    }
}

