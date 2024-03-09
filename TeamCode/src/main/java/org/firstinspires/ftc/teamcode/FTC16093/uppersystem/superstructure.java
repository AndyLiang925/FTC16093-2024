package org.firstinspires.ftc.teamcode.FTC16093.uppersystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.FTC16093.XCYBoolean;
import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;

@Config
public class superstructure {
    private DcMotorEx armDrive   = null;
    private DcMotorEx amlDrive   = null;
    private Servo gb1 = null;
    private Servo gb2 = null;
    private Servo wrt = null;

    private Runnable drive_period;
    private final LinearOpMode opMode;

    public superstructure(LinearOpMode opMode, Runnable drivePeriod){
        drive_period = drivePeriod;
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

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

        amlDrive.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);
        wrt.setDirection(Servo.Direction.FORWARD);
        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void grab1_close(){
        gb1.setPosition(0.53);
    }
    public void grab2_close(){
        gb2.setPosition(0.76);
    }
    public void grab1_open(){
        gb1.setPosition(0.22);
    }
    public void grab2_open(){
        gb2.setPosition(0.45);
    }
    public void wrist_grab_distal(){
        setArmPosition(200);
        setArmLength(1150);
        wrt.setPosition(0.23);
    }
    public void newSleep(int sleepTime){//tested sleep
        try { Thread.sleep(sleepTime); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }//test for sleeping
    }
    public void autoGrabUpward(){//UNTESTED 2+2 grab the top two pixels
        setArmPosition(100);
        newSleep(300);
        setArmLength(0);
        grab1_open();
        grab2_open();
        newSleep(300);
        wrt.setPosition(0.35);
        newSleep(300);
        grab1_close();
        grab2_close();
        newSleep(300);
        wrt.setPosition(1);
        setArmPosition(0);
    }
    public void wrist_grab_distalAuto(int armPos){
        setArmPosition(armPos);
        try { Thread.sleep(1000); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }//test for sleeping
        wrt.setPosition(0.23);
        setArmLength(1150);
    }
    public void wrist_grab_proximal(){
        setArmLength(0);
        wrt.setPosition(0.34);
        setArmPosition(0);
    }
    public void setArmLength(int length){
        amlDrive.setTargetPosition(length);
        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amlDrive.setPower(1);
    }
    public void setArmPosition(int pos){
        if(armDrive.getCurrentPosition()>pos){
            armDrive.setPower(0.6);
        }else{
            armDrive.setPower(0.8);
        }
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setPower(0.8);
    }
    public void setUp(){
        setArmPosition(0);
        setArmLength(10);
        gb1.setPosition(0.53);//gb1.setPosition(0.22);
        gb2.setPosition(0.76);//gb2.setPosition(0.45);
        wrt.setPosition(1);
    }
    public void wrist_to_middle(){
        wrt.setPosition(1); // initial 0.55
    }
    public void putOnSpikeMark(){
        wrt.setPosition(0.4);
        //newSleep(300);
        setArmPosition(20);
        newSleep(300);
        gb1.setPosition(0.22);
        newSleep(300);
        wrt.setPosition(0.7);
        newSleep(300);
        gb1.setPosition(0.53);
        newSleep(300);
        wrt.setPosition(1);
        newSleep(300);
    }
    public void putOnBackDrop(){
        setArmPosition(1890);
        newSleep(1000);
        //gb1.setPosition(0.22);
        grab2_open();
        grab1_open();
        newSleep(500);
        grab2_close();
        grab1_close();
        setUp();
    }
    public void intake2(int armPos){
        gb1.setPosition(0.53);
        newSleep(300);
        wrist_grab_distalAuto(armPos);
        newSleep(300);

        setArmPosition(armPos); // added: reset arm position
        newSleep(300);
        gb1.setPosition(0.22);

        newSleep(300);
        wrt.setPosition(1);
        newSleep(300);
        setArmLength(0);
        newSleep(300);
        setArmPosition(0);
        newSleep(300);
    }
}
