package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC16093.AutoMaster;

@Autonomous
public class AutoTestBlueNear extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = PROXIMAL;
        side_color = BLUE;///cnmjwh
        initHardware();
        setUpAuto();
        setKickProp();
        spikeMarkDump();
        putOnSpikeMark();
        backDropDump();
        sleep(1000);
        putOnBackDrop();
        setUpAuto();
        extraCredit();
        putOnBackDrop();
        setUpAuto();
    }

}

