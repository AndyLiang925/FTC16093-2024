package org.firstinspires.ftc.teamcode.FTC16093.uppersystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;

@Config
public class SuperStructure {
    private DcMotorEx armDrive   = null;
    private DcMotorEx amlDrive   = null;
    private Servo gb1 = null;
    private Servo gb2 = null;
    private Servo wrt = null;

    public int grab_left = 1, grab_right =-1;
    public static double grab1_close = 0.85,grab2_close = 0.48, grab_delta = 0.26;

    private int grab_side;

    public void setGrabSide(int grab_side) {
        this.grab_side = grab_side;
    }

    private final LinearOpMode opMode;
    public SuperStructure(LinearOpMode opMode){
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
        gb2.setPosition(grab2_close);
    }
    public void grab1_close(){
        gb1.setPosition(grab1_close);
    }
    public void grab2_open(){
        gb2.setPosition(grab2_close-grab_delta);
    }
    public void grab1_open(){
        gb1.setPosition(grab1_close-grab_delta);
    }

    public void grab1_openDrop(){
        gb1.setPosition(0.7);
    }

    public void releasePurple(int grab_side){ //side 1 grab_left //-1 grab_right
        if(grab_side == grab_left){
            grab2_open();
        } else if (grab_side == grab_right) {
            grab1_open();
        }
    }

    public void releaseYellow(int grab_side){
        if(grab_side == grab_left){
            grab1_open();
        } else if (grab_side == grab_right) {
            grab2_open();
        }
    }
    public void grabPurple (int grab_side){
        if(grab_side == grab_left){
            grab2_close();
        } else if (grab_side == grab_right) {
            grab1_close();
        }
    }
    public void grabYellow (int grab_side){
        if(grab_side == grab_left){
            grab1_close();
        } else if (grab_side == grab_right) {
            grab2_close();
        }
    }

    public void sleep(int sleepTime){//tested sleep
        try { Thread.sleep(sleepTime); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }//test for sleeping
    }

    public void putOnSpikeMark() {
        wristDown();
        setArmPosition(245);
        sleep(500);
        setArmLength(570,0.5);
        sleep(800);
        releasePurple(grab_side);
        sleep(200);
        wrist_to_middle();
        setArmLength(0,1);
        sleep(200);
        setArmPosition(0);
        sleep(800);
    }
    public void putOnBackDrop(){
        releaseYellow(grab_side);
        sleep(200);
    }

    public void release_extra(){
        releasePurple(grab_side);
        sleep(200);
    }

    public void drop_upward(){
        setArmPosition(2150);
        sleep(800);
        releasePurple(grab_side);
        sleep(300);
        wrist_to_upward_drop();
        sleep(200);
        set_wrist_pos(0.26);
        sleep(150);
        set_wrist_pos(0.35);
        sleep(500);
        setArmPosition_slow(2050);
    }

    public void autoGrabPrepare(int armPos_near){
        setArmPosition(armPos_near);
        sleep(300);
        setArmLength(0,1);
        releasePurple(grab_side);
        sleep(300);
        wrt.setPosition(0.51);
    }
    public void autoGrabFinish(){
        grabPurple(grab_side);
        sleep(300);
        wrist_to_middle();
        setArmPosition(0);
    }

    public void intake2_lowFar() {
        setArmPosition(230);
        wrt.setPosition(0.45);

        sleep(600);

        releasePurple(grab_side);
        sleep(500);
        setArmLength(560,1);
        sleep(800);

        setArmLength(250,1);
        setArmPosition(350);
        sleep(400);

        wrist_to_down();
        setArmLength(0,1);
        sleep(300);
        wrist_to_middle();
        sleep(300);
        setArmPosition(10);

        sleep(200);
        setArmLength(0,1);
        sleep(200);
        setArmPosition(0);
    }

    public void setArmLength(int length,double power){
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
    public void setArmPosition_slow(int pos){
        armDrive.setPower(0.3);
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public int getArmPosition(){
        return armDrive.getCurrentPosition();
    }

    public void wrist_to_middle(){
        wrt.setPosition(0.86); // initial 0.55
    }
    public void wrist_to_down(){//
        wrt.setPosition(0.34); // initial 0.55
    }
    public void wristDown(){
        wrt.setPosition(0.51);
    }
    public void wrist_to_upward(){
        wrt.setPosition(0.25); //0.28
    }
    public void wrist_to_upward_drop(){
        wrt.setPosition(0.32); //0.28
    }
    public void set_wrist_pos(double pos){
        wrt.setPosition(pos); //0.28
    }
}
