package org.firstinspires.ftc.teamcode.FTC16093;

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
        armDrive  = hardwareMap.get(DcMotorEx.class, "arm");
        amlDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrt = hardwareMap.get(Servo.class, "wrist");
        gb1 = hardwareMap.get(Servo.class, "grab2");
        gb2 = hardwareMap.get(Servo.class, "grab1");
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        startSide = DISTAL;
        side_color = RED;
        setArmLength(10);
        gb1.setPosition(0.53);
        gb2.setPosition(0.76);
        wrt.setPosition(1);
        initHardware();
        moveToCenter();
        ///退退退
        setArmPosition(170);
        sleep(300);
        setArmLength(-450);
        sleep(300);
        barkKickProp();
        setArmPosition(10);
        setArmLength(10);
        sleep(1000);
        //////
        wrt.setPosition(0.4);
        sleep(300);
        spikeMarkDump();
        sleep(1000);
        sleep(1000);
        setArmPosition(0);
        sleep(300);
        gb1.setPosition(0.22);

        sleep(1500);

        wrt.setPosition(0.7);
        sleep(200);
        gb1.setPosition(0.53);

        sleep(200);
        wrt.setPosition(1);
        DistalBackDropDump();
        setArmPosition(1850);
        sleep(1500);
        gb2.setPosition(0.45);
        //gb1.setPosition(0.22);
        sleep(1000);
        setArmPosition(0);
        gb1.setPosition(0.53);
        gb2.setPosition(0.76);
        sleep(1500);
        //parking();

        //wrt.setPosition(0.34);
        //extraCredit();
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

