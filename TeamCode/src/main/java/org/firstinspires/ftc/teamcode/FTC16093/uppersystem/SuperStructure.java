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
    private final DcMotorEx amlDrive;
    private final DcMotorEx armExternalEnc;
    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private PIDFController armPidCtrl;
    private Servo gb1 = null;
    private Servo gb2 = null;
    private Servo wrt = null;

    public int grab_left = 1, grab_right = -1;
    public static double grab1_close = 0.85, grab2_close = 0.48, grab_delta = 0.26;

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
        amlDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrt = hardwareMap.get(Servo.class, "wrist");
        gb1 = hardwareMap.get(Servo.class, "grab1");
        gb2 = hardwareMap.get(Servo.class, "grab2");

        armExternalEnc = hardwareMap.get(DcMotorEx.class, "hangRight");

        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        amlDrive.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);

        wrt.setDirection(Servo.Direction.FORWARD);

        armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void grab2_close() {
        gb2.setPosition(grab2_close);
    }

    public void grab1_close() {
        gb1.setPosition(grab1_close);
    }

    public void grab2_open() {
        gb2.setPosition(grab2_close - grab_delta);
    }

    public void grab1_open() {
        gb1.setPosition(grab1_close - grab_delta);
    }

    public void reset_arm_encoders() {
        armDrive.setPower(-0.2);
        sleep(500);
        armDrive.setPower(0);
        armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void grab1_drop() {
        gb1.setPosition(0.7);
    }
    public void grab2_drop(){
        gb2.setPosition(0.33);
    }

    public void releasePurple(int grab_side) { //side 1 grab_left //-1 grab_right
        if (grab_side == grab_left) {
            grab2_drop();
        } else if (grab_side == grab_right) {
            grab1_drop();
        }
    }

    public void releaseYellow(int grab_side) {
        if (grab_side == grab_left) {
            grab1_drop();
        } else if (grab_side == grab_right) {
            grab2_drop();
        }
    }

    public void grabPurple(int grab_side) {
        if (grab_side == grab_left) {
            grab2_close();
        } else if (grab_side == grab_right) {
            grab1_close();
        }
    }

    public void grabYellow(int grab_side) {
        if (grab_side == grab_left) {
            grab1_close();
        } else if (grab_side == grab_right) {
            grab2_close();
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
        wristDown();
        setArmPosition(470); //245
        sleep(500);
        setArmLength(570, 0.5);
        sleep(800);
        releasePurple(grab_side);
        sleep(200);
        setArmLength(0, 1);
        sleep(500);
        setArmPosition(0);
        wrist_to_middle();
        sleep(800);
    }

    public void putOnBackDrop() {
        wrist_to_middle();
        releaseYellow(grab_side);
        sleep(200);
    }

    public void release_extra() {
        releasePurple(grab_side);
        sleep(200);
    }

    public void drop_upward() {
        setArmPosition(4516);
        sleep(800);
        releaseYellow(grab_side);
        releasePurple(grab_side);
        sleep(300);
        wrist_to_upward_drop();
        sleep(200);
        set_wrist_pos(0.23);
        setArmPosition_slow(4200);
        sleep(500);
        setArmPosition(2000);
        wrist_to_middle();
    }

    public void autoGrabPrepare(int armPos_near) {
        setArmPosition(armPos_near);
        setArmLength(0, 1);
        releasePurple(grab_side);
        wrt.setPosition(0.51);
    }

    public void autoGrabFinish() {
        grabPurple(grab_side);
        sleep(300);
        wrist_to_middle();
        setArmPosition(0);
    }

    public void intake2_lowFar() {
        setArmPosition(470);
        grab1_open();
        grab2_open();
        wrt.setPosition(0.45);
        sleep(600);

        sleep(500);
        setArmLength(560, 1);
        sleep(800);
        grabYellow(grab_side);
        grabPurple(grab_side);
        sleep(200);

        setArmLength(250, 1);
        setArmPosition(500);
        sleep(400);

        wrist_to_down();
        setArmLength(0, 1);
        sleep(300);
        wrist_to_middle();
        sleep(300);
        setArmPosition(100);

        sleep(200);
        setArmLength(0, 1);
        sleep(200);
        setArmPosition(0);
    }

    public void setArmLength(int length, double power) {
        amlDrive.setTargetPosition(length);
        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amlDrive.setPower(0.85);
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
    public void resetArmLength(){
        amlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        amlDrive.setPower(-0.2);
        long end = System.currentTimeMillis() + 300;
        while (end > System.currentTimeMillis()) {
            opMode.idle();
        }
        amlDrive.setPower(0);
        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetArmPosition(){
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getArmPosition() {
        return armExternalEnc.getCurrentPosition();
    }

    private static final double WRIST_MIDDLE_POSITION = 0.86;
    public void wrist_to_middle() {
        wrt.setPosition(0.86); // initial 0.55
    }

    public void wrist_to_down() {//
        wrt.setPosition(0.34); // initial 0.55
    }

    public void wristDown() {
        wrt.setPosition(0.51);
    }

    public void wrist_to_upward() {
        wrt.setPosition(0.25); //0.28
    }

    public void wrist_to_upward_drop() {
        wrt.setPosition(0.32); //0.28
    }

    public void set_wrist_pos(double pos) {
        wrt.setPosition(pos); //0.28
    }

    private double armTargetPosition;

    public void update() {
        armDrive.setPower(armPidCtrl.update(armExternalEnc.getCurrentPosition() - armTargetPosition));
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
