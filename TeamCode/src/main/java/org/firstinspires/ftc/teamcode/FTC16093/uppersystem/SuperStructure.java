package org.firstinspires.ftc.teamcode.FTC16093.uppersystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SuperStructure {
    public static final double EXTERNAL_RATE = 1;
    private final DcMotorEx armDrive;
    private final DcMotorEx slideDrive;
    private final DcMotorEx armExternalEnc;
    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController armPidCtrl;
    private Servo grabRight = null;
    private Servo grabLeft = null;
    private Servo wrt = null;

    public int grab_left = 1, grab_right = -1;
    public static double grabRight_close = 0.78, grabLeft_close = 0.47, grab_openDelta = 0.28, grab_dropDelta = 0.2;
    public static double wrist_origin = 0.77, wrist_intakeNear = 0.57, wrist_intakeFar = 0.53, wrist_upwardDrop = 0.48, wrist_drop = 0.8;

    private int grab_side;

    public void setGrabSide(int grab_side) {
        this.grab_side = grab_side;
    }

    private final LinearOpMode opMode;

    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure(LinearOpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);
        armDrive = hardwareMap.get(DcMotorEx.class, "arm");
        slideDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrt = hardwareMap.get(Servo.class, "wrist");
        wrt = hardwareMap.get(Servo.class, "wrist");
        grabRight = hardwareMap.get(Servo.class, "grabRight");
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");

        armExternalEnc = hardwareMap.get(DcMotorEx.class, "hangRight");

        slideDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideDrive.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);

        wrt.setDirection(Servo.Direction.FORWARD);
        grabRight.setDirection(Servo.Direction.REVERSE);

        armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void grabLeft_close() {
        grabLeft.setPosition(grabLeft_close);
    }

    public void grabRight_close() {
        grabRight.setPosition(grabRight_close);
    }

    public void grabLeft_open() {
        grabLeft.setPosition(grabLeft_close - grab_openDelta);
    }

    public void grabRight_open() {
        grabRight.setPosition(grabRight_close - grab_openDelta);
    }

    public void grabRight_drop() {
        grabRight.setPosition(grabRight_close - grab_dropDelta);
    }
    public void grabLeft_drop(){
        grabLeft.setPosition(grabLeft_close - grab_dropDelta);
    }

    public void openPurple(int grab_side) { 
        if (grab_side == grab_left) {
            grabLeft_open();
        } else if (grab_side == grab_right) {
            grabRight_open();
        }
    }
    
    public void releasePurple(int grab_side) { //side 1 grab_left //-1 grab_right
        if (grab_side == grab_left) {
            grabLeft_drop();
        } else if (grab_side == grab_right) {
            grabRight_drop();
        }
    }

    public void releaseYellow(int grab_side) {
        if (grab_side == grab_left) {
            grabRight_drop();
        } else if (grab_side == grab_right) {
            grabLeft_drop();
        }
    }

    public void grabPurple(int grab_side) {
        if (grab_side == grab_left) {
            grabLeft_close();
        } else if (grab_side == grab_right) {
            grabRight_close();
        }
    }

    public void grabYellow(int grab_side) {
        if (grab_side == grab_left) {
            grabRight_close();
        } else if (grab_side == grab_right) {
            grabLeft_close();
        }
    }

    public void sleep(int sleepTime) {
        long end = System.currentTimeMillis() + sleepTime;
        while (opMode.opModeIsActive() && end > System.currentTimeMillis() && updateRunnable != null) {
            updateRunnable.run();
        }
    }

    /**
     * 放置地面预载
     */
    public void putOnSpikeMark() {
        wristGround();
//        setArmPosition(470);
//        sleep(300);
        setSlide(570, 0.8);
        sleep(400);
        openPurple(grab_side);
        sleep(200);
        setSlide(0, 1);
//        sleep(500);
//        setArmPosition(0);
        wrist_origin();
    }

    public void putOnBackDrop() {
        releaseYellow(grab_side);
    }

    public void release_extra() {
        releasePurple(grab_side);
        sleep(200);
    }

    public void drop_upward() {
        sleep(200);
        releaseYellow(grab_side);
        releasePurple(grab_side);
        setSlide(100,0.8);
        sleep(100);
        setArmPosition(4100);
        sleep(500);
    }
    public void toOrigin(){
        setArmPosition(0);
        setSlide(0,1);
        sleep(200);
        wrt.setPosition(wrist_origin);
        sleep(1000);
        grabLeft_close();
        grabRight_close();
    }
    public void dropToOrigin(){
        setArmPosition(0);
        setSlide(0,0.89);
        wrist_origin();
    }

    public void autoGrabPrepare(int armPos_near) {
        setArmPosition(armPos_near);
        setSlide(0, 1);
        openPurple(grab_side);
        wrt.setPosition(0.55);
    }

    public void autoGrabFinish() {
        grabPurple(grab_side);
        sleep(300);
        wrist_origin();
        setArmPosition(0);
    }

    public void intake2_lowFar() {
        setArmPosition(470);
        grabRight_open();
        grabLeft_open();
        wrist_intakeFar();
        sleep(300);

        setSlide(560, 1);
        sleep(800);
        grabYellow(grab_side);
        grabPurple(grab_side);
        sleep(200);

        setSlide(0, 1);
        wrist_upwardDrop();
        sleep(200);
        setArmPosition(0);
        wrist_origin();
    }

    public void intakeFar_prep(){
        setArmPosition(470);
        grabRight_open();
        grabLeft_open();
        wrist_intakeFar();
        sleep(300);
        setSlide(573, 0.9);
    }
    public void intakeFar_grab(){
        grabYellow(grab_side);
        grabPurple(grab_side);
        sleep(100);
        setArmPosition(500);
        wrist_upwardDrop();
        setSlide(0, 1);
        sleep(200);
        //setArmPosition(0);
        wrist_origin();
    }

    public void setSlide(int length, double power) {
        slideDrive.setTargetPosition(length);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDrive.setPower(0.85);
    }

    public void setArmPosition(int pos) {
        armTargetPosition = EXTERNAL_RATE * pos;
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(armExternalEnc.getCurrentPosition() <= 800 && pos <= armExternalEnc.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.3,0.3);
        }else if(armExternalEnc.getCurrentPosition() < 3000 && pos >= armExternalEnc.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.8,0.8);
        }else if(pos >= armExternalEnc.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.5,0.5);
        }else{
            armPidCtrl.setOutputBounds(-0.8,0.8);
        }
    }

    public void setArmPosition_slow(int pos) {
        armTargetPosition = pos * EXTERNAL_RATE;
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPidCtrl.setOutputBounds(-0.7,0.7);
    }

    // ToDo: arm length reset
    public void resetSlide(){
        slideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideDrive.setPower(-0.3);
        long end = System.currentTimeMillis() + 300;
        while (end > System.currentTimeMillis()) {
            opMode.idle();
        }
        slideDrive.setPower(0);
        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetArm(){
        armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDrive.setPower(-0.3);
        long end = System.currentTimeMillis() + 300;
        while (end > System.currentTimeMillis()) {
            opMode.idle();
        }
        armDrive.setPower(0);
        armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getArmPosition() {
        return armExternalEnc.getCurrentPosition();
    }

    private static final double WRIST_MIDDLE_POSITION = 0.86;
    public void wrist_origin() {
        wrt.setPosition(wrist_origin);
    }
    public void wrist_drop (){
        wrt.setPosition(wrist_drop);
    }

    public void wrist_intakeFar() {
        wrt.setPosition(wrist_intakeFar);
    }

    public void wristGround() {
        wrt.setPosition(wrist_intakeNear);
    }

    public void wrist_upwardDrop() {
        wrt.setPosition(wrist_upwardDrop);
    }
    private double armTargetPosition;

    public void update() {
        armDrive.setPower(armPidCtrl.update(armExternalEnc.getCurrentPosition() - armTargetPosition));
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
