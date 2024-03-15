package org.firstinspires.ftc.teamcode.FTC16093.teleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.FTC16093.XCYBoolean;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;

import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

//Assets from Qualcomm

import java.util.List;
@TeleOp
public class OpenDrive16093 extends LinearOpMode {//
    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel\
    private DcMotorSimple hangLeft   = null;
    private DcMotorSimple hangRight   = null;
    private DcMotorEx armDrive   = null;
    private DcMotorEx amlDrive   = null;
    public static String motor_name_0 = "hangLeft";
    public static String motor_name_1 = "hangRight";
    private Servo gb1 = null;
    private Servo brake = null;
    private Servo gb2 = null;
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
    public double driver_speed=1.0;//手动慢速档
    public double rotation_speed=1.0;//旋转降速
    private BarkMecanumDrive bdrive;

    enum Sequence {
        AIM, RELEASE, RUN
    }
    int mode=0;
    private OpenDrive16093.Sequence sequence;
    @Override//
    public void runOpMode() throws InterruptedException {
        time = NanoClock.system();
        //get all Lynx Module device objects
        allHubs = hardwareMap.getAll(LynxModule.class);

        //init position
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double wrtp = 0.23;
        double plnp=0.2;//plane pos
        double backPower=-0.2;//the power going back
        double brkp;//brake's pos
        int amlp=0;
        int armp=0;
        int index=0;
        int maxIndex=6;
        int minIndex=0;
        int pd=0;//判断手腕是否手动微调
        int pdArm=0;//判断大臂是否微调
        int armLengthLevels[] = {50,400,630,750,950,1100};
        int armPosLevels[] = {1820,1814,1747,1740,1740,1730};
        double wrtLevels[] = {1,1,1,1,1,1};
        boolean leftGrabOpen=false;
        boolean rightGrabOpen=false;
        boolean colorSensorUsed=false;
        XCYBoolean aim =new XCYBoolean(()->gamepad2.a);
        XCYBoolean distal = new XCYBoolean(()->gamepad2.dpad_up);
        XCYBoolean proximal = new XCYBoolean(()->gamepad2.dpad_down);
        XCYBoolean drop = new XCYBoolean(()->gamepad2.y);
        XCYBoolean brake_start = new XCYBoolean(()->gamepad1.b);
        //XCYBoolean back = new XCYBoolean(()->gamepad2.x);
        XCYBoolean leftGrab = new XCYBoolean(()->gamepad2.left_trigger>0);
        XCYBoolean rightGrab = new XCYBoolean(()->gamepad2.right_trigger>0);
        XCYBoolean humanGrab = new XCYBoolean(()->gamepad2.back);
        XCYBoolean toRun = new XCYBoolean(()->gamepad2.x);
        XCYBoolean armBack = new XCYBoolean(()->gamepad2.left_bumper);
        XCYBoolean armExpandBack = new XCYBoolean(()->gamepad2.right_bumper);
        XCYBoolean TankForward = new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean TankBackward = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean TankLeftward = new XCYBoolean(()->gamepad1.dpad_left);
        XCYBoolean TankRightward = new XCYBoolean(()->gamepad1.dpad_right);
        XCYBoolean hangLower = new XCYBoolean(()->gamepad1.left_bumper);
        XCYBoolean hangUp = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean plane_shoot = new XCYBoolean(()->gamepad1.x);
        XCYBoolean movePixel = new XCYBoolean(()->gamepad2.left_stick_button);
        XCYBoolean slowMode = new XCYBoolean(()->gamepad1.right_trigger>0||gamepad1.left_trigger>0);
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rearRight");
        hangLeft  = hardwareMap.get(DcMotorSimple.class, motor_name_0);
        hangRight = hardwareMap.get(DcMotorSimple.class, motor_name_1);
        armDrive  = hardwareMap.get(DcMotorEx.class, "arm");
        amlDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrt = hardwareMap.get(Servo.class, "wrist");
        gb1 = hardwareMap.get(Servo.class, "grab2");
        gb2 = hardwareMap.get(Servo.class, "grab1");
        brake = hardwareMap.get(Servo.class, "brake");
        plane = hardwareMap.get(Servo.class, "plane");
        imu = hardwareMap.get(IMU.class, "imu");
//        if(slowMode.get()){
//            speed=0.3;
//        }else{
//            speed=1;
//        }

        //手动慢速
        //parameter box
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        hangLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        hangRight.setDirection(DcMotorSimple.Direction.REVERSE);
        amlDrive.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);
        wrt.setDirection(Servo.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        hangLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        hangRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sequence = Sequence.RUN;
        gb1.setPosition(0.53);
        gb2.setPosition(0.76);
        plnp=0.2;
        plane.setPosition(plnp);
        brkp=0.5;
        brake.setPosition(brkp);

        waitForStart();
        while (opModeIsActive()){

            logic_period();
            if(brake_start.toTrue()){
                if(brkp==0.31){
                    brkp=0.5;
                }else{
                    brkp=0.29;
                }
                //brkp=0.31;
                brake.setPosition(brkp);
            }
            if(slowMode.get()){
                driver_speed=0.7;
            }else{
                driver_speed=1;
            }
            if(armBack.toTrue()) {
                armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armDrive.setPower(backPower);
                sleep_with_drive(200);
                armDrive.setPower(0);
                armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(plane_shoot.get()){
                plnp=0.5;
            }
            if(armExpandBack.toTrue()) {
                amlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                amlDrive.setPower(backPower);
                sleep_with_drive(200);
                amlDrive.setPower(0);
                amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad1.a){
                imu.resetYaw();
            }
                if(toRun.toTrue()){
                    setArmLength(0);
                    setArmPosition(0);
                    wrtp=1;
                    wrt.setPosition(wrtp);
                    sequence=OpenDrive16093.Sequence.RUN;
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
                    sequence=OpenDrive16093.Sequence.RELEASE;
                    telemetry.addData("release",0);
                }
                if(aim.toTrue()){
                    setArmLength(0);
                    setArmPosition(0);
                    wrtp=0.34;
                    wrt.setPosition(wrtp);
                    sequence=OpenDrive16093.Sequence.AIM;
                    telemetry.addData("aim",0);
                }
                if(humanGrab.toTrue()){
                    colorSensorUsed=false;
                }
                if(sequence==OpenDrive16093.Sequence.AIM){
                    speed = 0.5;
                    if(distal.toTrue()){
                        setArmPosition(200);
                        sleep_with_drive(200);
                        setArmLength(1150);
                        wrtp=0.23;
                        wrt.setPosition(wrtp);
                    }
                    if(proximal.toTrue()){
                        setArmLength(0);
                        wrtp=0.34;
                        wrt.setPosition(wrtp);
                        setArmPosition(0);
                    }

//                    if(leftGrab.toTrue()){
//                        gb1.setPosition(leftGrabOpen?0.22:0.53);
//                        leftGrabOpen=!leftGrabOpen;
//                    }
//                    if(rightGrab.toTrue()){
//                        gb2.setPosition(rightGrabOpen?0.76:0.45);
//                        rightGrabOpen=!rightGrabOpen;
//                    }
                    gb1.setPosition(leftGrab.get()?0.22:0.53);
                    gb2.setPosition(rightGrab.get()?0.45:0.76);
                }
                if(sequence==OpenDrive16093.Sequence.RUN){
                    setArmLength(0);
                    setArmPosition(0);
                    wrtp=1;
                    wrt.setPosition(wrtp);
                    speed=1;
                    gb1.setPosition(0.53);
                    gb2.setPosition(0.76);
                }
                if(mode==1&&armDrive.getCurrentPosition()>armPosLevels[index]-300){
                    setArmLength(armLengthLevels[index]);
                    mode=0;
                }
                if(sequence==OpenDrive16093.Sequence.RELEASE) {
                    speed = 0.4;
                    if (distal.toTrue()) {
                        index = index + 1 >= maxIndex - 1 ? maxIndex - 1 : index + 1;
                        mode=1;
                        pd=0;
                        pdArm=0;
                    }
                    if (proximal.toTrue()) {
                        index = index - 1 < minIndex ? minIndex : index - 1;
                        mode=1;
                        pd=0;
                        pdArm=0;
                    }
                    if (gamepad2.right_stick_y>0) {
                        armp=armp+((int)gamepad2.right_stick_y*15)>2300?2300:armp+((int)gamepad2.right_stick_y*15);
                        pdArm=1;
                        rotation_speed=0.4;
                    }else if(gamepad2.right_stick_y<0){
                        armp=armp+((int)gamepad2.right_stick_y*15)<0?0:armp+((int)gamepad2.right_stick_y*15);
                        pdArm=1;
                        rotation_speed=0.4;
                    }else if(pdArm==0){
                        armp=armPosLevels[index];
                        rotation_speed=1;
                    }else{
                        rotation_speed=1;
                    }
                    setArmPosition(armp);



                    if (gamepad2.left_stick_y>0) {
                        wrtp=wrtp+(gamepad2.left_stick_y/50)>1?1:wrtp+(gamepad2.left_stick_y/50);
                        pd=1;
                    }else if(gamepad2.left_stick_y<0){
                        wrtp=wrtp+(gamepad2.left_stick_y/50)<0?0:wrtp+(gamepad2.left_stick_y/50);
                        pd=1;
                    }else if(pd==0){
                        wrtp=wrtLevels[index];
                    }
                    if(movePixel.toTrue()){
                        wrtp=0.30;
                        pd=1;
                    }
                    wrt.setPosition(wrtp);
                    gb1.setPosition(leftGrab.get()?0.22:0.53);
                    gb2.setPosition(rightGrab.get()?0.45:0.76);
//                    if (leftGrab.toTrue()) {
//                        gb1.setPosition(leftGrabOpen?0.22:0.53);
//                        leftGrabOpen = !leftGrabOpen;
////                        setArmPosition(armPosLevels[index]-130);//
////                        sleep(300);
////                        setArmPosition(armPosLevels[index]);
//                    }
//                    if (rightGrab.toTrue()) {
//                        gb2.setPosition(rightGrabOpen?0.76:0.45);
//                        rightGrabOpen = !rightGrabOpen;
////                        setArmPosition(armPosLevels[index]-130);
////                        sleep(300);
////                        setArmPosition(armPosLevels[index]);
//                    }
                }
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

            wrt.setPosition(wrtp);
            plane.setPosition(plnp);
            telemetry.addData("操作模式:", sequence==OpenDrive16093.Sequence.AIM?"aim1":(sequence==OpenDrive16093.Sequence.RUN?"run":(sequence==OpenDrive16093.Sequence.RELEASE?"drop":"null")));
            telemetry.addData("大臂伸长:",amlDrive.getCurrentPosition());
            telemetry.addData("大臂位置:",armDrive.getCurrentPosition());
            telemetry.addData("底盘速度:",speed);
            telemetry.addData("手腕位置:",wrtp);
            telemetry.addData("index:",index);
//            telemetry.addData("leftFront_velo",bdrive.getMotorVelo(1));
//            telemetry.addData("leftBack_velo",bdrive.getMotorVelo(2));
//            telemetry.addData("rightFront_velo",bdrive.getMotorVelo(3));
//            telemetry.addData("rightBack_velo",bdrive.getMotorVelo(4));

            telemetry.update();
            if(gamepad1.dpad_up||gamepad1.dpad_down||gamepad1.dpad_right||gamepad1.dpad_left){
                bark_tank_drive_period();
            }else{
                bark_drive_period();
            }
        }
    }
    private void logic_period() {
        //IMPORTANT: READ ALL
        XCYBoolean.bulkRead();
        //time of this period
        period_time_sec = time.seconds() - last_time_sec;
        //P.S. telemetry comes from parent class, caption = title
//        telemetry.addData("elapse time", period_time_sec * 1000);
        last_time_sec = time.seconds();
        //send data
//        telemetry.update();
        //clear read(ed) cache
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
    public void setArmLength(int length){
        amlDrive.setTargetPosition(length);
        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amlDrive.setPower(1);
    }
    public void setArmPosition(int pos){
        if(armDrive.getCurrentPosition()>pos){
            armDrive.setPower(0.7);
        }else{
            armDrive.setPower(0.9);
        }
        armDrive.setTargetPosition(pos);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setPower(0.8);

    }
    public void bark_drive_period(){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = (gamepad1.right_stick_x) * 0.7;
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
    public void bark_tank_drive_period(){
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        if(gamepad1.dpad_up){
            frontLeftPower = 1;
            backLeftPower = 1;
            frontRightPower = 1;
            backRightPower = 1;
        }
        if(gamepad1.dpad_down){
            frontLeftPower = -1;
            backLeftPower = -1;
            frontRightPower = -1;
            backRightPower = -1;
        }
        if(gamepad1.dpad_right){
            frontLeftPower = 1;
            backLeftPower = -1;
            frontRightPower = -1;
            backRightPower = 1;
        }
        if(gamepad1.dpad_left){
            frontLeftPower = -1;
            backLeftPower = 1;
            frontRightPower = 1;
            backRightPower = -1;
        }
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
