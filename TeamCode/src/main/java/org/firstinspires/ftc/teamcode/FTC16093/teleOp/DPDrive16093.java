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
    double wrist_pos = 0.77;
    // 手腕位置
    double wrist_origin=0.77, wrist_intakeNear=0.57, wrist_intakeFar= 0.54;
    // 爪子位置
    private double grabRight_open=0.5, grabRight_drop =0.63, grabRight_grab = 0.815, grabRight_close = 1;
    private double grabLeft_open=0.19, grabLeft_drop = 0.33, grabLeft_grab=0.49, grabLeft_close = 0.69;
    private ElapsedTime runtime = new ElapsedTime();
    enum Sequence {
        AIM, RELEASE, RUN, MOVEPIXEL
    }
    int mode=0;
    private Sequence sequence;

    View relativeLayout;
    @Override//
    public void runOpMode() throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        time = NanoClock.system();
        //get all Lynx Module device objects
        allHubs = hardwareMap.getAll(LynxModule.class);

        double plane_close = 0.6, plane_open = 0.9;//plane pos
        double backPower=-0.3;//the power going back
        double brkp;//brake's pos

        int armp=0;
        int index=0;
        int indexPixel = 0;
        int maxIndex=10;
        int minIndex=0;

        // 放片档位
        int slideLevel[] = {0,179,257,337,431,520,234,323,452,570}; //337 431
        int armLevel[] = {2155,2050,1980,1930,1920,1922,1994,1981,1951,1934};
        double wrtLevels[] = {0.8,0.75,0.80,0.80,0.80,0.80,0.49,0.49,0.49,0.5};

        // 调片档位
        int slidePixelLevel[] = {0,47,120,187,268,351,456,578,507,512};
        int armPixelLevel[] = {2200,2130,2100,2050,2020,2000,1950,1920,1830,1760};
        double wristPixelLevel[] = {0.6,0.6,0.6,0.6,0.6,0.61,0.62,0.63,0.65,0.65};

        boolean rightGrabOpen=false;
        boolean leftGrabOpen=false;
        boolean remove_limit=false;

        XCYBoolean aim =new XCYBoolean(()->gamepad2.a);
        XCYBoolean distal = new XCYBoolean(()->gamepad2.dpad_up);
        XCYBoolean proximal = new XCYBoolean(()->gamepad2.dpad_down);
        XCYBoolean drop = new XCYBoolean(()->gamepad2.y);
        XCYBoolean armBack = new XCYBoolean(()->gamepad2.left_bumper);
        XCYBoolean slideBack = new XCYBoolean(()->gamepad2.right_bumper);
        XCYBoolean movePixel = new XCYBoolean(()->gamepad2.x);
        
        XCYBoolean brake_start = new XCYBoolean(()->gamepad1.b);
        XCYBoolean rightGrab = new XCYBoolean(()->gamepad1.right_trigger>0);
        XCYBoolean leftGrab = new XCYBoolean(()->gamepad1.left_trigger>0);
        XCYBoolean toRun = new XCYBoolean(()->gamepad1.left_stick_button);
        
        XCYBoolean dropUp = new XCYBoolean(()-> sequence == Sequence.RELEASE && gamepad1.dpad_up);
        XCYBoolean dropLower = new XCYBoolean(()-> sequence == Sequence.RELEASE && gamepad1.dpad_down);

        XCYBoolean pixelUp = new XCYBoolean(()-> sequence == Sequence.MOVEPIXEL && gamepad1.dpad_up);
        XCYBoolean pixelLower = new XCYBoolean(()-> sequence == Sequence.MOVEPIXEL && gamepad1.dpad_down);

        XCYBoolean hangLower = new XCYBoolean(()->gamepad1.left_bumper);
        XCYBoolean hangUp = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean plane_shoot = new XCYBoolean(()->gamepad1.y);
        
        XCYBoolean dpad = new XCYBoolean(()-> gamepad1.dpad_left||gamepad1.dpad_up||gamepad1.dpad_down);
        

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
        plane.setPosition(plane_close);

        brkp=0.5;
        brake.setPosition(brkp);
        heading_target=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        waitForStart();

        while (opModeIsActive()){
            logic_period();
            // 重置无头
            if(gamepad1.a){
                imu.resetYaw();
            }
            // 取消纸飞机和悬挂的锁： 一操的左右trigger
            if(remove_endgame_limit.get()){
                remove_limit=true;
            }
            // 发射纸飞机: gamepad1.y
            if(plane_shoot.get()&&(runtime.seconds()>90||remove_limit)){
                plane.setPosition(plane_open);
            }
            // 悬挂上升或下放
            if(hangLower.get()){
                hangLeft.setPower(1);
                hangRight.setPower(1);
            }else if(hangUp.get()){
                hangLeft.setPower(-1);
                hangRight.setPower(-1);
            }else{
                hangLeft.setPower(0);
                hangRight.setPower(0);
            }
            // 悬挂卡住
            if(brake_start.toTrue()&&(runtime.seconds()>90||remove_limit)){
                if(brkp==0.5){
                    brkp=0.93;
                }else{
                    brkp=0.5;
                }
                brake.setPosition(brkp);
            }
            // 大臂下转：二操 left_bumper
            if(armBack.toTrue()) {
                armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armDrive.setPower(backPower);
                sleep_with_drive(200);
                armDrive.setPower(0);
                armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // 收回滑轨： 二操 right_bumper
            if(slideBack.toTrue()) {
                slideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideDrive.setPower(backPower);
                sleep_with_drive(200);
                slideDrive.setPower(0);
                slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // 回到初始位置： 一操 left_stick_button
            if(toRun.toTrue()){
                if(sequence == Sequence.AIM) {
                    grabRight.setPosition(grabRight_grab);
                    grabLeft.setPosition(grabLeft_grab);
                    sleep_with_drive(300);
                    setSlide(0);
                    speed = 1;
                }
                sleep_with_drive(300);
                if(sequence== Sequence.RELEASE){
                    wrist_pos = wrist_origin;
                    wrt.setPosition(wrist_pos);
                    speed = 1;
                    setArmPosition(1200);
                    sleep_with_drive(500);
                }
                setSlide(0);
                setArmPosition(0);

                if(sequence== Sequence.RELEASE){
                    speed=1;
                    sleep_with_drive(500);
                }

                wrist_pos = wrist_origin;
                wrt.setPosition(wrist_pos);
                sequence= Sequence.RUN;
                telemetry.addData("run",0);
            }


            if(drop.toTrue()){
                mode=1;
                sequence= Sequence.RELEASE;
                telemetry.addData("release",0);
                rightGrabOpen = false;
                leftGrabOpen = false;
                setArmPosition(armp);
                sleep_with_drive(1000);
                setSlide(slideLevel[index]);

            }

            if(aim.toTrue()){
                setSlide(0);
                setArmPosition(0);
                wrist_pos= wrist_intakeNear;
                wrt.setPosition(wrist_pos);
                sequence= Sequence.AIM;
                telemetry.addData("aim",0);

                rightGrabOpen=true;
                leftGrabOpen=true;
                grabRight.setPosition(grabRight_open);
                grabLeft.setPosition(grabLeft_open);
                grabRight.setPosition(rightGrabOpen?grabRight_open:grabRight_grab);
                grabLeft.setPosition(leftGrabOpen?grabLeft_open:grabLeft_grab);
            }

            // 底盘移动
            if(sequence== Sequence.RUN){
                setSlide(0);
                sleep_with_drive(100);
                setArmPosition(0);
                wrist_pos=wrist_origin;
                wrt.setPosition(wrist_pos);
                speed=1;
                grabRight.setPosition(grabRight_grab);
                grabLeft.setPosition(grabLeft_grab);
                
                if(distal.toTrue()){
                    sequence= Sequence.AIM;
                    telemetry.addData("aim",0);
                }
                if(proximal.toTrue()){
                    sequence= Sequence.AIM;
                    telemetry.addData("aim",0);
                }
            }

            // 抓片
            if(sequence== Sequence.AIM){

                if(distal.toTrue()){
                    speed = 0.5;
                    setArmPosition(124);
                    wrist_pos = wrist_intakeFar;
                    wrt.setPosition(wrist_pos);
                    sleep_with_drive(200);
                    setSlide(185);
                    rightGrabOpen=true;
                    leftGrabOpen=true;
                    grabRight.setPosition(grabRight_open);
                    grabLeft.setPosition(grabLeft_open);

                    setArmPosition(235);
                    sleep_with_drive(200);
                    setSlide(573);
                }

                if(proximal.toTrue()){
                    speed=1;
                    setSlide(0);
                    wrist_pos=wrist_intakeNear;
                    wrt.setPosition(wrist_pos);
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

            // 放片
            if(sequence== Sequence.RELEASE) {
                speed = 0.4;
                if (dropUp.toTrue()) {
                    index = index + 1 >= maxIndex - 1 ? maxIndex - 1 : index + 1;
                    mode=1;
                }
                if (dropLower.toTrue()) {
                    index = index - 1 < minIndex ? minIndex : index - 1;
                    mode=1;
                }
                indexPixel = index;
                armp= armLevel[index];
                rotation_speed=1;
                setArmPosition(armp);


                if(movePixel.toTrue()){
                    grabRight.setPosition(grabRight_close);
                    grabLeft.setPosition(grabLeft_close);
                    sequence = Sequence.MOVEPIXEL;
                }
                wrist_pos = wrtLevels [index];
                wrt.setPosition(wrist_pos);

                if (leftGrab.toTrue()) {
                    rightGrabOpen = !rightGrabOpen;
                    grabRight.setPosition(rightGrabOpen?grabRight_drop:grabRight_grab);
                }
                if (rightGrab.toTrue()) {
                    leftGrabOpen = !leftGrabOpen;
                    grabLeft.setPosition(leftGrabOpen?grabLeft_drop:grabLeft_grab);
                }
            }

            // 挪动Pixel
            if (sequence == Sequence.MOVEPIXEL){
                speed = 0.4;
                if (pixelUp.toTrue()) {
                    indexPixel = indexPixel + 1 >= maxIndex - 1 ? maxIndex - 1 : indexPixel + 1;
                    mode=0;
                }
                if (pixelLower.toTrue()) {
                    indexPixel = indexPixel - 1 < minIndex ? minIndex : indexPixel - 1;
                    mode=0;
                }

                setArmPosition(armPixelLevel[indexPixel]);
                setSlide(slidePixelLevel[indexPixel]);
                wrist_pos = wristPixelLevel [indexPixel];
                wrt.setPosition(wristPixelLevel[indexPixel]);
            }

            // 滑轨到指定位置
            if(mode==1 && armDrive.getCurrentPosition()>armLevel[index]-300){
                setSlide(slideLevel[index]);
                mode=0;
            }

            telemetry.addData("run_time :", runtime.seconds());
            telemetry.addData("imu_degrees :",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("target_degrees :",heading_target);
            telemetry.addData("操作模式:", sequence== Sequence.AIM?"aim1":(sequence== Sequence.RUN?"run":(sequence== Sequence.RELEASE?"drop":"null")));//drive mode
            telemetry.addData("大臂伸长:",slideDrive.getCurrentPosition());//arm expand position
            telemetry.addData("大臂位置:",armDrive.getCurrentPosition());//arm position
            telemetry.addData("底盘速度:",speed);// robot speed
            telemetry.addData("手腕位置:",wrist_pos);//wrist position
            telemetry.addData("index:",index);//level of upper system

            telemetry.update();
            if(dpad.toTrue()){
                heading_target=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            wrt.setPosition(wrist_pos);
            // 底盘移动
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
    public void setSlide(int length){
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