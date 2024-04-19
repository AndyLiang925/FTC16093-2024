package org.firstinspires.ftc.teamcode.FTC16093.uppersystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;
import org.firstinspires.ftc.teamcode.util.Encoder;

import XCYOS.DelayTask;
import XCYOS.InstantTask;
import XCYOS.Task;
import XCYOS.TaskChainBuilder;

@Config
public class NewSuperStructure {
    enum Grabber {
        RIGHT,
        LEFT,
        ALL,
        PURPLE,
        YELLOW
    }

    private boolean sideIsRed;
    private final DcMotorEx armDrive;
    private final DcMotorEx amlDrive;
    private final DcMotorEx armExternalEnc;
    private final Servo gb1, gb2, wrt;

    public int grab_left = 1, grab_right = -1;
    public static double grab1_close = 0.85, grab2_close = 0.48, grab_delta = 0.26;
    private int grab_side;

    public void setGrabSide(int grab_side) {
        this.grab_side = grab_side;
    }


    public void setSideIsRed(boolean sideIsRed) {
        this.sideIsRed = sideIsRed;
    }

    public NewSuperStructure(HardwareMap hardwareMap) {
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

        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Task toInitPosition;
    public Task resetArmEnc;

    public void initTasks() {
        TaskChainBuilder tb = new TaskChainBuilder();
        resetArmEnc = tb.start()
                .add(() -> armDrive.setPower(-0.2))
                .add(new DelayTask(500))
                .add(() -> {
                    armDrive.setPower(0);
                    armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                })
                .end().getBase();
        toInitPosition = tb.start()
                .add(() -> {
                    setArmPosition(0);
                    setArmLength(-5, 1);
                    grab1_close();//gb1.setPosition(0.22);
                    grab2_close();//gb2.setPosition(0.45);
                    wrist_to_middle();
                })
                .end().getBase();
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

    public Task grab(Grabber grabber){
        return new InstantTask(()->{
            switch (grabber) {
                case ALL:
                    grab1_close();
                    grab2_close();
                case LEFT:
                    grab2_close();
                case RIGHT:
                    grab1_close();
                case PURPLE:
                    if (sideIsRed) {
                        grab2_close();
                    } else {
                        grab1_close();
                    }
                case YELLOW:
                    if (sideIsRed) {
                        grab1_close();
                    } else {
                        grab2_close();
                    }
        }});
    }

    public Task putOnGround(){
        TaskChainBuilder tb = new TaskChainBuilder();
        return tb.start()
                .add(()->{
                    setArmPosition(245);
                    wristDown();
                })
                .add(new DelayTask(150))
                .add(()->{
                    setArmLength(570, 0.5);
                }).end().getBase();
    }

    public Task release(Grabber grabber) {
        TaskChainBuilder tb = new TaskChainBuilder();

        return tb.start()
                .add(() -> {
                    switch (grabber) {
                        case ALL:
                            grab1_open();
                            grab2_open();
                        case LEFT:
                            grab2_open();
                        case RIGHT:
                            grab1_open();
                        case PURPLE:
                            if (sideIsRed) {
                                grab2_open();
                            } else {
                                grab1_open();
                            }
                        case YELLOW:
                            if (sideIsRed) {
                                grab1_open();
                            } else {
                                grab2_open();
                            }
                    }
                })
                .add(new DelayTask(300))
                .add(() -> {
                    setArmPosition(0);
                    setArmLength(0, 1);
                })
                .add(new DelayTask(300))
                .add(this::wrist_to_middle)
                .end().getBase();
    }

//    public Task toBackDrop(){
//        TaskChainBuilder tb = new TaskChainBuilder();
//        return
//    }

//    public void putOnBackDrop() {
//        wrt.setPosition(0.95);
//        releaseYellow(grab_side);
//        sleep(200);
//    }


//    public void autoGrabPrepare(int armPos_near) {
//        setArmPosition(armPos_near);
//        setArmLength(0, 1);
//        releasePurple(grab_side);
//        wrt.setPosition(0.51);
//    }


//    public void intake2_lowFar() {
//        setArmPosition(230);
//        wrt.setPosition(0.45);
//
//        sleep(600);
//
//        releasePurple(grab_side);
//        sleep(500);
//        setArmLength(560, 1);
//        sleep(800);
//
//        setArmLength(250, 1);
//        setArmPosition(350);
//        sleep(400);
//
//        wrist_to_down();
//        setArmLength(0, 1);
//        sleep(300);
//        wrist_to_middle();
//        sleep(300);
//        setArmPosition(10);
//
//        sleep(200);
//        setArmLength(0, 1);
//        sleep(200);
//        setArmPosition(0);
//    }

    public void setArmLength(int length, double power) {
        amlDrive.setTargetPosition(length);
        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amlDrive.setPower(1);
    }

    public void setArmPosition(int pos) {
        if (armDrive.getCurrentPosition() <= 200 && pos <= armDrive.getCurrentPosition()) {
            armDrive.setPower(0.75);
        } else if (armDrive.getCurrentPosition() < 1300) {
            armDrive.setPower(1);
        } else {
            armDrive.setPower(0.7);
        }
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmPosition_slow(int pos) {
        armDrive.setPower(0.3);
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getArmPosition() {
        return armDrive.getCurrentPosition();
    }

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
}
