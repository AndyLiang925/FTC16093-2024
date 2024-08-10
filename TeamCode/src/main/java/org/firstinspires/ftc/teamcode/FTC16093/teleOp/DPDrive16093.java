package org.firstinspires.ftc.teamcode.FTC16093.teleOp;

import android.app.Activity;
import android.view.View;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.FTC16093.XCYBoolean;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;

import java.util.List;

@TeleOp
public class DPDrive16093 extends LinearOpMode {//
    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel\
    private DcMotorEx hangLeft   = null;
    private DcMotorEx hangRight   = null;
    private DcMotorEx armDrive   = null;
    private DcMotorEx slideDrive   = null;
    private Servo grabRight = null;
    private Servo brake = null;
    private Servo grabLeft = null;
    private Servo wrt = null;
    private Servo plane = null;
    private NanoClock time;
    private List<LynxModule> allHubs;
    private Pose2d current_pos, lastEjectPos;
    private IMU imu;
    private double last_time_sec;
    //time of this period (sec)
    private double period_time_sec;
    public double speed=1.0;
    public double driver_speed = 1.0;//手动慢速档 driver controlled slow mode speed
    public double rotation_speed=1.0;//旋转降速 extra slow speed during rotation
    private double heading_target;//heading target
    private double grabRight_open=0.47, grabRight_drop = 0.67, grabRight_grab = 0.87, grabRight_close = 0.97;
    private double grabLeft_open=0.13, grabLeft_drop = 0.43, grabLeft_grab=0.53, grabLeft_close = 0.73;
    private ElapsedTime runtime = new ElapsedTime();
    enum Sequence {
        AIM, RELEASE, RUN
    }
    int mode=0;
    private DPDrive16093.Sequence sequence;

    View relativeLayout;
    @Override//
    public void runOpMode() throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        time = NanoClock.system();
        //get all Lynx Module device objects
        allHubs = hardwareMap.getAll(LynxModule.class);
        //init position
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double wrtp = 1;
        double wrist_origin=0.77, wrist_intakeNear=0.57, wrist_intakeFar= 0.53, wrist_movePixel = 0.4;

        double plnp= 0.6;//plane pos
        double backPower=-0.3;//the power going back
        double brkp;//brake's pos
        int amlp=0;
        int armp=0;
        int index=0;
        int maxIndex=10;
        int minIndex=0;
        int pd=0;//判断手腕是否手动微调 whether wrist is in driver controlled mode
        int pdArm=0;//判断大臂是否微调 whether arm is in driver controlled mode
        int armLengthLevels[] = {0,179,257,337,431,520,234,323,452,570}; //337 431
        int armPosLevels[] = {2155,2050,1980,1930,1920,1922,1994,1981,1951,1934};
        double wrtLevels[] = {0.95,0.96,1,1,1,1,0.31,0.31,0.31,0.354};
        boolean rightGrabOpen=false;
        boolean leftGrabOpen=false;
        boolean remove_limit=false;

        XCYBoolean aim =new XCYBoolean(()->gamepad2.a);
        XCYBoolean distal = new XCYBoolean(()->gamepad2.dpad_up);
        XCYBoolean proximal = new XCYBoolean(()->gamepad2.dpad_down);
        XCYBoolean drop = new XCYBoolean(()->gamepad2.y);
        XCYBoolean brake_start = new XCYBoolean(()->gamepad1.b);
        XCYBoolean rightGrab = new XCYBoolean(()->gamepad1.right_trigger>0);
        XCYBoolean leftGrab = new XCYBoolean(()->gamepad1.left_trigger>0);
        XCYBoolean toRun = new XCYBoolean(()->gamepad1.left_stick_button);
        XCYBoolean armBack = new XCYBoolean(()->gamepad2.left_bumper);
        XCYBoolean armExpandBack = new XCYBoolean(()->gamepad2.right_bumper);
        XCYBoolean armDropUp = new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean armDropDown = new XCYBoolean(()->gamepad1.dpad_down);

        XCYBoolean hangLower = new XCYBoolean(()->gamepad1.left_bumper);
        XCYBoolean hangUp = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean plane_shoot = new XCYBoolean(()->gamepad1.y);
        XCYBoolean movePixel = new XCYBoolean(()->gamepad2.left_stick_button);
        XCYBoolean dpad = new XCYBoolean(()->gamepad1.dpad_left||gamepad1.dpad_up||gamepad1.dpad_down);

        XCYBoolean remove_endgame_limit = new XCYBoolean(()->gamepad1.right_trigger>0&&gamepad1.left_trigger>0);
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rearRight");
        hangLeft  = hardwareMap.get(DcMotorEx.class, "hangLeft");
        hangRight = hardwareMap.get(DcMotorEx.class, "hangRight");
        armDrive  = hardwareMap.get(DcMotorEx.class, "arm");
        slideDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrt = hardwareMap.get(Servo.class, "wrist");
        grabRight = hardwareMap.get(Servo.class, "grabRight");
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        brake = hardwareMap.get(Servo.class, "brake");
        plane = hardwareMap.get(Servo.class, "plane");
        imu = hardwareMap.get(IMU.class, "imu");

        //parameter box
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        hangLeft.setDirection(DcMotorEx.Direction.FORWARD);
        hangRight.setDirection(DcMotorEx.Direction.REVERSE);
        slideDrive.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);
        wrt.setDirection(Servo.Direction.FORWARD);
        grabRight.setDirection(Servo.Direction.REVERSE);
        brake.setDirection(Servo.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sequence = Sequence.RUN;

        grabRight.setPosition(grabRight_grab);
        grabLeft.setPosition(grabLeft_grab);
        wrt.setPosition(wrist_origin);
        plnp=0.6;
        plane.setPosition(plnp);
        brkp=0.5;
        brake.setPosition(brkp);
        heading_target=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        waitForStart();

        while (opModeIsActive()){
            logic_period();

            if(gamepad1.a){
                imu.resetYaw();
            }
            if(remove_endgame_limit.get()){
                remove_limit=true;
            }
            if(brake_start.toTrue()&&(runtime.seconds()>90||remove_limit)){
                if(brkp==0.5){
                    brkp=0.93;
                }else{
                    brkp=0.5;
                }
                brake.setPosition(brkp);
            }
            if(plane_shoot.get()&&(runtime.seconds()>90||remove_limit)){
                plnp=0.9;
            }

            if(armBack.toTrue()) {
                armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armDrive.setPower(backPower);
                sleep_with_drive(200);
                armDrive.setPower(0);
                armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(armExpandBack.toTrue()) {
                slideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideDrive.setPower(backPower);
                sleep_with_drive(200);
                slideDrive.setPower(0);
                slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(toRun.toTrue()){
                if(sequence == Sequence.AIM) {
                    grabRight.setPosition(grabRight_grab);
                    grabLeft.setPosition(grabLeft_grab);
                }
                sleep_with_drive(100);
                if(sequence== Sequence.AIM){
                    setArmLength(0);
                }
                if(sequence== Sequence.RELEASE){
                    wrtp = wrist_origin;
                    wrt.setPosition(wrtp);
                    speed = 1;
                    setArmPosition(1200);
                    sleep_with_drive(500);
                }
                if(sequence== Sequence.AIM&&wrtp==0.45){
                    speed=1;
                    sleep_with_drive(500);
                }
                setArmLength(0);
                setArmPosition(0);

                if(sequence== Sequence.RELEASE){
                    speed=1;
                    sleep_with_drive(500);
                }

                wrtp=wrist_origin;
                wrt.setPosition(wrtp);
                sequence= DPDrive16093.Sequence.RUN;
                telemetry.addData("run",0);
            }
            if(drop.toTrue()){
                mode=1;
                setArmLength(0);
                setArmPosition(1900);
                wrtp=0;
                wrt.setPosition(wrtp);
                pd=0;
                pdArm=0;
                sequence= DPDrive16093.Sequence.RELEASE;
                telemetry.addData("re lease",0);
                rightGrabOpen=false;
                leftGrabOpen=false;
                grabRight.setPosition(rightGrabOpen?0.6:grabRight_grab);
                grabLeft.setPosition(leftGrabOpen?0.22:grabLeft_grab);
            }

            if(aim.toTrue()){
                setArmLength(0);
                setArmPosition(0);
                wrtp= wrist_intakeNear;
                wrt.setPosition(wrtp);
                sequence= DPDrive16093.Sequence.AIM;
                telemetry.addData("aim",0);

                rightGrabOpen=true;
                leftGrabOpen=true;
                grabRight.setPosition(grabRight_open);
                grabLeft.setPosition(grabLeft_open);
                grabRight.setPosition(rightGrabOpen?grabRight_open:grabRight_grab);
                grabLeft.setPosition(leftGrabOpen?grabLeft_open:grabLeft_grab);
            }

            if(sequence== DPDrive16093.Sequence.AIM){
                speed = 0.5;
                if(distal.toTrue()){
                    setArmPosition(230);
                    rightGrabOpen=true;
                    leftGrabOpen=true;
                    sleep_with_drive(200);
                    grabRight.setPosition(grabRight_open);
                    grabLeft.setPosition(grabLeft_open);
                    setArmLength(570);
                    wrtp=wrist_intakeFar;
                    wrt.setPosition(wrtp);
                }
                if(proximal.toTrue()){
                    setArmLength(0);
                    wrtp=wrist_intakeNear;
                    wrt.setPosition(wrtp);
                    setArmPosition(0);
                    sleep_with_drive(200);
                    rightGrabOpen=true;
                    leftGrabOpen=true;
                    grabRight.setPosition(grabRight_open);
                    grabLeft.setPosition(grabLeft_open);
                }

                if (leftGrab.toTrue()) {
                    leftGrabOpen = !leftGrabOpen;
                    grabLeft.setPosition(leftGrabOpen?grabLeft_open:grabLeft_grab); //grabLeft_open
                }
                if (rightGrab.toTrue()) {
                    rightGrabOpen = !rightGrabOpen;
                    grabRight.setPosition(rightGrabOpen?grabRight_open:grabRight_grab); //grabRight_open
                }
            }

            if(sequence== DPDrive16093.Sequence.RUN){
                setArmLength(0);
                setArmPosition(0);
                wrtp=wrist_origin;
                wrt.setPosition(wrtp);
                speed=1;
                grabRight.setPosition(grabRight_grab);
                grabLeft.setPosition(grabLeft_grab);
                if(distal.toTrue()){
                    sequence= DPDrive16093.Sequence.AIM;
                    telemetry.addData("aim",0);
                }
                if(proximal.toTrue()){
                    sequence= DPDrive16093.Sequence.AIM;
                    telemetry.addData("aim",0);
                }
            }
            if(mode==1&&armDrive.getCurrentPosition()>armPosLevels[index]-300){
                setArmLength(armLengthLevels[index]);
                mode=0;
            }

            if(sequence== DPDrive16093.Sequence.RELEASE) {
                speed = 0.4;
                if (armDropUp.toTrue()) {
                    index = index + 1 >= maxIndex - 1 ? maxIndex - 1 : index + 1;
                    mode=1;
                    pd=0;
                    pdArm=0;
                }
                if (armDropDown.toTrue()) {
                    index = index - 1 < minIndex ? minIndex : index - 1;
                    mode=1;
                    pd=0;
                    pdArm=0;
                }
                if (gamepad2.right_stick_y<0) {
                    armp=armp-((int)gamepad2.right_stick_y*15)>2300?2300:armp-((int)gamepad2.right_stick_y*15);
                    pdArm=1;
                    rotation_speed=0.5;
                }else if(gamepad2.right_stick_y>0){
                    armp=armp-((int)gamepad2.right_stick_y*15)<0?0:armp-((int)gamepad2.right_stick_y*15);
                    pdArm=1;
                    rotation_speed=0.5;
                }else if(pdArm==0){
                    armp=armPosLevels[index];
                    rotation_speed=1;
                }else{
                    rotation_speed=1;
                }
                setArmPosition(armp);

                if (gamepad2.left_stick_y<0) {
                    wrtp=wrtp-(gamepad2.left_stick_y/50)>1?1:wrtp-(gamepad2.left_stick_y/50);
                    pd=1;
                }else if(gamepad2.left_stick_y>0){
                    wrtp=wrtp-(gamepad2.left_stick_y/50)<0?0:wrtp-(gamepad2.left_stick_y/50);
                    pd=1;
                }else if(pd==0){
                    wrtp=wrtLevels[index];
                }

                if(movePixel.toTrue()){
                    wrtp=0.5;
                    pd=1;
                    grabRight.setPosition(grabRight_close);
                    grabLeft.setPosition(grabLeft_close);
                }
                wrt.setPosition(wrtp);
                if (leftGrab.toTrue()) {
                    rightGrabOpen = !rightGrabOpen;
                    grabRight.setPosition(rightGrabOpen?grabRight_drop:grabRight_grab);
                    grabLeft.setPosition(leftGrabOpen?grabLeft_drop:grabLeft_grab);
                }
                if (rightGrab.toTrue()) {
                    leftGrabOpen = !leftGrabOpen;
                    grabRight.setPosition(rightGrabOpen?grabRight_drop:grabRight_grab);
                    grabLeft.setPosition(leftGrabOpen?grabLeft_drop:grabLeft_grab);
                }
            }

            if(hangLower.get()){
                wrtp=0.9;
                hangLeft.setPower(1);
                hangRight.setPower(1);
            }else if(hangUp.get()){
                //wrtp=0.53;
                hangLeft.setPower(-1);
                hangRight.setPower(-1);
            }else{
                hangLeft.setPower(0);
                hangRight.setPower(0);
            }
            wrt.setPosition(wrtp);
            plane.setPosition(plnp);

            telemetry.addData("run_time :", runtime.seconds());
            telemetry.addData("imu_degrees :",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("target_degrees :",heading_target);
            telemetry.addData("操作模式:", sequence== DPDrive16093.Sequence.AIM?"aim1":(sequence== DPDrive16093.Sequence.RUN?"run":(sequence== DPDrive16093.Sequence.RELEASE?"drop":"null")));//drive mode
            telemetry.addData("大臂伸长:",slideDrive.getCurrentPosition());//arm expand position
            telemetry.addData("大臂位置:",armDrive.getCurrentPosition());//arm position
            telemetry.addData("底盘速度:",speed);// robot speed
            telemetry.addData("手腕位置:",wrtp);//wrist position
            telemetry.addData("index:",index);//level of upper system

            telemetry.update();

            if(dpad.toTrue()){
                heading_target=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            bark_drive_period();
        }
    }

    private void logic_period() {
        //IMPORTANT: READ ALL
        XCYBoolean.bulkRead();
        //time of this period//
        period_time_sec = time.seconds() - last_time_sec;
        //P.S. telemetry comes from parent class, caption = title
//        telemetry.addData("elapse time", period_time_sec * 1000);/
        last_time_sec = time.seconds();//
        //send data//
//        telemetry.update();
        //clear read(ed) cache////
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
    public void setArmLength(int length){
        slideDrive.setTargetPosition(length);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDrive.setPower(0.85);
    }
    public void setArmPosition(int pos){
        if(armDrive.getCurrentPosition()<=400&&pos<=armDrive.getCurrentPosition()){
            armDrive.setPower(0.3);
        }else if(armDrive.getCurrentPosition()<1300&&pos>=armDrive.getCurrentPosition()){
            armDrive.setPower(1);
        }else if(pos>=armDrive.getCurrentPosition()){
            armDrive.setPower(0.6);
        }else{
            armDrive.setPower(1);
        }
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void bark_drive_period(){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = (gamepad1.right_stick_x) * 0.5;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) - Math.abs(rx), 1);
        double frontLeftPower = ((rotY + rotX - rx)) / denominator;
        double backLeftPower = ((rotY - rotX - rx)) / denominator;
        double frontRightPower = ((rotY - rotX + rx)) / denominator;
        double backRightPower = ((rotY + rotX + rx)) / denominator;
        leftFrontDrive.setPower(frontLeftPower*speed*driver_speed*rotation_speed);
        leftBackDrive.setPower(backLeftPower*speed*driver_speed*rotation_speed);
        rightFrontDrive.setPower(frontRightPower*speed*driver_speed*rotation_speed);
        rightBackDrive.setPower(backRightPower*speed*driver_speed*rotation_speed);
    }

    public void sleep_with_drive(double time_mm) {
        long start_time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start_time < time_mm) {
            bark_drive_period();
        }
    }
}