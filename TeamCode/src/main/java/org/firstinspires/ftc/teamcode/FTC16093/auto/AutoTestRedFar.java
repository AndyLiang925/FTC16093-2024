package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTestRedFar extends AutoMaster {
    private DcMotorEx armDrive   = null;
    private DcMotorEx amlDrive   = null;
    private Servo gb1 = null;
    private Servo gb2 = null;
    private Servo wrt = null;
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = RED;
        initHardware();
        spikeMarkDump();
        putOnSpikeMark();
        DistalBackDropDump();
        putOnBackDrop();
        sleep(1000);
        setUpAuto();
    }
    public void setArmLength(int length){
        amlDrive.setTargetPosition(length);
        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amlDrive.setPower(1);
    }
    public void setArmPosition(int pos){
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setPower(0.8);

    }
}

