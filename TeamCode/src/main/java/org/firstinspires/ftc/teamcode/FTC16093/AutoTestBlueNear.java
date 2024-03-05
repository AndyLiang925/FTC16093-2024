package org.firstinspires.ftc.teamcode.FTC16093;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTestBlueNear extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = PROXIMAL;
        side_color = BLUE;
        setUpAuto();
        initHardware();
        spikeMarkDump();

        putOnSpikeMark();
        backDropDump();
        putOnBackDrop();
        setUpAuto();

        extraCredit();
        putOnBackDrop();
        setUpAuto();
    }

}

