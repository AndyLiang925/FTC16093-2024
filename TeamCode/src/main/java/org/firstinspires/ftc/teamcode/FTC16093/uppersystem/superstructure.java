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
        gb1 = hardwareMap.get(Servo.class, "grab1");
        gb2 = hardwareMap.get(Servo.class, "grab2");
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
    public void grab2_close(){
        gb1.setPosition(0.66);
    }
    public void grab1_close(){
        gb2.setPosition(0.48);
    }
    public void grab2_open(){
        gb1.setPosition(0.91);
    }
    public void grab1_open(){
        gb2.setPosition(0.23);
    }
    public void wrist_grab_distal(){
        setArmPosition(200);
        setArmLength(1150);
        wrt.setPosition(0.23);
    }//
    public void newSleep(int sleepTime){//tested sleep
        try { Thread.sleep(sleepTime); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }//test for sleeping
    }
    public void autoGrabUpward(int armPos_near){//UNTESTED 2+2 grab the top two pixels
        setArmPosition(armPos_near);
        newSleep(300);
        setArmLength(0);
        grab1_open();

        newSleep(300);
        wrt.setPosition(0.23);
        newSleep(300);
        grab1_close();
        newSleep(300);
        wrt.setPosition(1);
        setArmPosition(0);
    }

    public void autoGrabPrepare(int armPos_near){
        setArmPosition(armPos_near);
        newSleep(300);
        setArmLength(0);
        grab1_open();

        newSleep(300);
        wrt.setPosition(0.51);
        newSleep(300);
    }
    public void autoGrabFinish(){
        grab1_close();
        newSleep(300);
        wrt.setPosition(1);
        setArmPosition(0);
    }
    public void wrist_grab2_distalAuto(int armPos){
        setArmPosition(armPos);
        //try { Thread.sleep(1000); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }//test for sleeping
        newSleep(600);
        grab2_open();
        newSleep(300);
        wrt.setPosition(0.45);
        newSleep(100);
        setArmLength(400);
    }
    public void wrist_grab1_distalAuto(int armPos){
        setArmPosition(armPos);
        newSleep(600);
        grab1_open();
        newSleep(300);
        wrt.setPosition(0.51);
        newSleep(100);
        setArmLength(432);
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
        if(armDrive.getCurrentPosition()<=200&&pos<=armDrive.getCurrentPosition()){
            armDrive.setPower(0.75);
        }else if(armDrive.getCurrentPosition()<1300){
            armDrive.setPower(1);
        }else{
            armDrive.setPower(0.7);
        }
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void wrist_to_middle(){
        wrt.setPosition(0.9); // initial 0.55
    }
    public void wrist_to_down(){
        wrt.setPosition(0.34); // initial 0.55
    }
    public void wristDown(){
        wrt.setPosition(0.51);
    }

    public void putOnBackDrop(){
        setArmPosition(1890);
        newSleep(1500);
        //gb1.setPosition(0.22);
        grab2_open();

        newSleep(200);
        grab2_close();
    }
    public void init_armExpand(){
        setArmLength(-20);
        newSleep(500);
        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
