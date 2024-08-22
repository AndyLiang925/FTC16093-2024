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
    public static double grabRight_grab = 0.76, grabLeft_grab = 0.48, grab_openDelta = 0.36, grab_dropDelta = 0.17,grab_closeDelta = 0.21;
    public static double wrist_origin = 0.77, wrist_intakeNear = 0.56, wrist_intakeFar = 0.53, wrist_upwardDrop = 0.50, wrist_drop = 0.8, wrist_pixelSlide = 0.43;

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


    public void grabLeft_open() {
        grabLeft.setPosition(grabLeft_grab - grab_openDelta);
    }

    public void grabRight_open() {
        grabRight.setPosition(grabRight_grab - grab_openDelta);
    }

    public void grabRight_drop() {
        grabRight.setPosition(grabRight_grab - grab_dropDelta);
    }
    public void grabLeft_drop(){
        grabLeft.setPosition(grabLeft_grab - grab_dropDelta);
    }

    public void grabRight_grab() {
        grabRight.setPosition(grabRight_grab);
    }
    public void grabLeft_grab(){
        grabLeft.setPosition(grabLeft_grab);
    }
    public void grabRight_close(){
        grabRight.setPosition(grabRight_grab + grab_closeDelta);
    }
    public void grabLeft_close(){
        grabLeft.setPosition(grabLeft_grab + grab_closeDelta);
    }

    public void closeYellow(int grab_side){
        if (grab_side == grab_left) {
            grabRight_close();
        } else if (grab_side == grab_right) {
            grabLeft_close();
        }
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
            grabLeft_grab();
        } else if (grab_side == grab_right) {
            grabRight_grab();
        }
    }

    public void grabYellow(int grab_side) {
        if (grab_side == grab_left) {
            grabRight_grab();
        } else if (grab_side == grab_right) {
            grabLeft_grab();
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
        setArmPosition(500);
        wristGround();
        setSlide(570, 0.8);
        sleep(600);
        openPurple(grab_side);
        sleep(200);
        setSlide(0, 1);
        wrist_origin();
    }

    public void dropYellow() {
        releaseYellow(grab_side);
        sleep(300);
    }

    public void release_extra() {
        releasePurple(grab_side);
        sleep(200);
    }

    public void drop_upward() {
        releaseYellow(grab_side);
        releasePurple(grab_side);
        sleep(200);
        setSlide(100,0.8);
        sleep(130);
        setArmPosition(4080);
        sleep(300);
    }
    public void toOrigin(){
        setArmPosition(0);
        setSlide(0,1);
        sleep(200);
        wrt.setPosition(wrist_origin);
        sleep(1000);
        grabLeft_grab();
        grabRight_grab();
    }

    public void dropToOrigin(){
        setArmPosition(0);
        setSlide(0,0.9);
        wrist_origin();
    }

    public void grabWhite1_Prepare(int armPos_near) {
        setArmPosition(armPos_near);
        openPurple(grab_side);
        wrt.setPosition(0.6);
    }
    public void grabWhite1_Finish() {
        grabPurple(grab_side);
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
        setArmPosition(800);
        wrist_pixelSlide();
        setSlide(0, 1);
    }

    public void intakeSide_prep(){
        setArmPosition(470);
        openPurple(grab_side);
        closeYellow(grab_side);
        wrist_intakeFar();
        sleep(300);
        setSlide(573, 0.9);
    }
    public void setSlide(int length, double power) {
        slideDrive.setTargetPosition(length);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDrive.setPower(0.9);
    }

    public void setArmPosition(int pos) {
        armTargetPosition = EXTERNAL_RATE * pos;
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(armExternalEnc.getCurrentPosition() <= 800 && pos <= armExternalEnc.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.3,0.3);
        }else if(armExternalEnc.getCurrentPosition() < 3000 && pos >= armExternalEnc.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.9,0.9);
        }else if(pos >= armExternalEnc.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.5,0.5);
        }else{
            armPidCtrl.setOutputBounds(-0.8,0.8);
        }
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
    public void wrist_pixelSlide(){
        wrt.setPosition(wrist_pixelSlide);
    }
    private double armTargetPosition;

    public void update() {
        armDrive.setPower(armPidCtrl.update(armExternalEnc.getCurrentPosition() - armTargetPosition));
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
