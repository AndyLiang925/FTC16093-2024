//package org.firstinspires.ftc.teamcode.FTC16093.teleOp;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.FTC16093.XCYBoolean;
//import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
//
//import java.util.List;
//
//@TeleOp
//public class DP_DropUpward extends LinearOpMode {
//    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
//    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel//
//    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
//    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel\
//    private DcMotorSimple hangLeft   = null;
//    private DcMotorSimple hangRight   = null;
//    private DcMotorEx armDrive   = null;
//    private DcMotorEx amlDrive   = null;
//    public static String motor_name_0 = "hangLeft";
//    public static String motor_name_1 = "hangRight";
//    private Servo gb1 = null;
//    private Servo brake = null;
//    private Servo gb2 = null;
//    private Servo wrt = null;
//    private Servo plane = null;
//    private NanoClock time;
//    private List<LynxModule> allHubs;
//    private Pose2d current_pos, lastEjectPos;
//    private IMU imu;
//    private double last_time_sec;
//    //time of this period (sec)
//    private double period_time_sec;
//    public double speed=1.0;
//    private BarkMecanumDrive drive;
//
//    enum Sequence {
//        AIM, RELEASE, RUN
//    }
//    int mode=0;
//    private DP_DropUpward.Sequence sequence;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        time = NanoClock.system();
//        //get all Lynx Module device objects
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        drive = new BarkMecanumDrive(hardwareMap);
//
//        telemetry.addData("leftFront_velo",drive.getMotorVelo(1));
//        telemetry.addData("leftBack_velo",drive.getMotorVelo(2));
//        telemetry.addData("rightFront_velo",drive.getMotorVelo(3));
//        telemetry.addData("rightBack_velo",drive.getMotorVelo(4));
//
//        //init position
//        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
//        double wrtp = 0.23;
//        double plnp=0.2;//plane pos
//        double backPower=-0.2;//the power of going back
//        double brkp;//brake's pos
//        int amlp=0;
//        int armp=0;
//        int index=0;
//        int maxIndex=6;
//        int minIndex=0;
//        int armLengthLevels[] = {50,400,630,750,950,1100};
//        int armPosLevels[] = {1920,1914,1847,1840,1840,1830};
//        double wrtLevels[] = {1,1,1,1,1,1};
//        boolean leftGrabOpen=false;
//        boolean rightGrabOpen=false;
//        boolean colorSensorUsed=false;
//        XCYBoolean aim =new XCYBoolean(()->gamepad2.a);
//        XCYBoolean distal = new XCYBoolean(()->gamepad2.dpad_up);
//        XCYBoolean proximal = new XCYBoolean(()->gamepad2.dpad_down);
//        XCYBoolean drop = new XCYBoolean(()->gamepad2.y);
//        XCYBoolean brake_start = new XCYBoolean(()->gamepad1.b);
//        //XCYBoolean back = new XCYBoolean(()->gamepad2.x);
//        XCYBoolean leftGrab = new XCYBoolean(()->gamepad2.left_trigger>0);
//        XCYBoolean rightGrab = new XCYBoolean(()->gamepad2.right_trigger>0);
//        XCYBoolean humanGrab = new XCYBoolean(()->gamepad2.back);
//        XCYBoolean toRun = new XCYBoolean(()->gamepad2.x);
//        XCYBoolean armBack = new XCYBoolean(()->gamepad2.left_bumper);
//        XCYBoolean armExpandBack = new XCYBoolean(()->gamepad2.right_bumper);
//        XCYBoolean TankForward = new XCYBoolean(()->gamepad1.dpad_up);
//        XCYBoolean TankBackward = new XCYBoolean(()->gamepad1.dpad_down);
//        XCYBoolean TankLeftward = new XCYBoolean(()->gamepad1.dpad_left);
//        XCYBoolean TankRightward = new XCYBoolean(()->gamepad1.dpad_right);
//        XCYBoolean hangLower = new XCYBoolean(()->gamepad1.left_bumper);
//        XCYBoolean hangUp = new XCYBoolean(()->gamepad1.right_bumper);
//        XCYBoolean plane_shoot = new XCYBoolean(()->gamepad1.x);
//        XCYBoolean slowMode = new XCYBoolean(()->gamepad1.right_trigger>0||gamepad1.left_trigger>0);
//        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
//        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "rearLeft");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rearRight");
//        hangLeft  = hardwareMap.get(DcMotorSimple.class, motor_name_0);
//        hangRight = hardwareMap.get(DcMotorSimple.class, motor_name_1);
//        armDrive  = hardwareMap.get(DcMotorEx.class, "arm");
//        amlDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
//        wrt = hardwareMap.get(Servo.class, "wrist");
//        gb1 = hardwareMap.get(Servo.class, "grab2");
//        gb2 = hardwareMap.get(Servo.class, "grab1");
//        brake = hardwareMap.get(Servo.class, "brake");
//        plane = hardwareMap.get(Servo.class, "plane");
//        imu = hardwareMap.get(IMU.class, "imu");
////        if(slowMode.get()){
////            speed=0.3;
////        }else{
////            speed=1;
////        }
//        //手动慢速
//        //parameter box
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        hangLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        hangRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        amlDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        armDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        wrt.setDirection(Servo.Direction.FORWARD);
//        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////        hangLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////        hangRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sequence = Sequence.RUN;
//        gb1.setPosition(0.53);
//        gb2.setPosition(0.76);
//        plnp=0.2;
//        plane.setPosition(plnp);
//        brkp=0.5;
//        brake.setPosition(brkp);
//        waitForStart();
//        while (opModeIsActive()){
//            logic_period();
//            if(brake_start.toTrue()){
//                if(brkp==0.31){
//                    brkp=0.5;
//                }else{
//                    brkp=0.29;
//                }
//                //brkp=0.31;
//                brake.setPosition(brkp);
//            }
//            if (gamepad2.left_stick_y>0) {
//                wrtp=wrtp+(gamepad2.left_stick_y/50)>0.72?0.72:wrtp+(gamepad2.left_stick_y/50);
//            }else if(gamepad2.left_stick_y<0){
//                wrtp=wrtp+(gamepad2.left_stick_y/50)<0.09?0.09:wrtp+(gamepad2.left_stick_y/50);
//            }
//            if(armBack.toTrue()) {
//                armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                armDrive.setPower(backPower);
//                sleep_with_drive(200);
//                armDrive.setPower(0);
//                armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            if(plane_shoot.get()){
//                plnp=0.5;
//            }
//            if(armExpandBack.toTrue()) {
//                amlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                amlDrive.setPower(backPower);
//                sleep_with_drive(200);
//                amlDrive.setPower(0);
//                amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//
//            if(gamepad1.a){
//                imu.resetYaw();
//            }
//            if(toRun.toTrue()){
//                setArmLength(0);
//                setArmPosition(0);
//                wrtp=1;
//                wrt.setPosition(wrtp);
//                sequence= DP_DropUpward.Sequence.RUN;
//                telemetry.addData("run",0);
//            }
//            if(drop.toTrue()){
//                mode=1;
//                setArmLength(0);
//                setArmPosition(1900);
//                wrtp=0;
//                wrt.setPosition(wrtp);
//                sequence= DP_DropUpward.Sequence.RELEASE;
//                telemetry.addData("release",0);
//            }
//            if(aim.toTrue()){
//                setArmLength(0);
//                setArmPosition(0);
//                wrtp=0.34;
//                wrt.setPosition(wrtp);
//                sequence= DP_DropUpward.Sequence.AIM;
//                telemetry.addData("aim",0);
//            }
//            if(humanGrab.toTrue()){
//                colorSensorUsed=false;
//            }
//            if(sequence== DP_DropUpward.Sequence.AIM){
//                speed = 0.45;
//                if(distal.toTrue()){
//                    setArmPosition(200);
//                    sleep_with_drive(200);
//                    setArmLength(1150);
//                    wrtp=0.23;
//                    wrt.setPosition(wrtp);
//                }
//                if(proximal.toTrue()){
//                    setArmLength(0);
//                    wrtp=0.34;
//                    wrt.setPosition(wrtp);
//                    setArmPosition(0);
//                }
//
//                if(leftGrab.toTrue()){
//                    gb1.setPosition(leftGrabOpen?0.22:0.53);
//                    leftGrabOpen=!leftGrabOpen;
//                }
//                if(rightGrab.toTrue()){
//                    gb2.setPosition(rightGrabOpen?0.76:0.45);
//                    rightGrabOpen=!rightGrabOpen;
//                }
//            }
//            if(sequence== DP_DropUpward.Sequence.RUN){
//                setArmLength(0);
//                setArmPosition(0);
//                wrtp=1;
//                wrt.setPosition(wrtp);
//                speed=1;
//                gb1.setPosition(0.53);
//                gb2.setPosition(0.76);
//            }
//            if(mode==1&&armDrive.getCurrentPosition()>armPosLevels[index]-300){
//                setArmLength(armLengthLevels[index]);
//                mode=0;
//            }
//            if(sequence== DP_DropUpward.Sequence.RELEASE) {
//                speed = 0.3;
//                if (distal.toTrue()) {
//                    index = index + 1 >= maxIndex - 1 ? maxIndex - 1 : index + 1;
//                    mode=1;
//                }
//                if (proximal.toTrue()) {
//                    index = index - 1 < minIndex ? minIndex : index - 1;
//                    mode=1;
//                }
//
//                setArmPosition(armPosLevels[index]);
//                bark_wrist_balancer();
//                //bark_wrist_balancer();
//                //wrt.setPosition(wrtp);
//                if (leftGrab.toTrue()) {
//                    gb1.setPosition(leftGrabOpen?0.22:0.53);
//                    leftGrabOpen = !leftGrabOpen;
////                        setArmPosition(armPosLevels[index]-130);
////                        sleep(300);
////                        setArmPosition(armPosLevels[index]);
//                }
//                if (rightGrab.toTrue()) {
//                    gb2.setPosition(rightGrabOpen?0.76:0.45);
//                    rightGrabOpen = !rightGrabOpen;
////                        setArmPosition(armPosLevels[index]-130);
////                        sleep(300);
////                        setArmPosition(armPosLevels[index]);
//                }
//            }
//            if(hangLower.get()){
//                hangLeft.setPower(1);
//                hangRight.setPower(1);
//            }else if(hangUp.get()){
//                hangLeft.setPower(-1);
//                hangRight.setPower(-1);
//            }else{
//                hangLeft.setPower(0);
//                hangRight.setPower(0);
//            }
//
//            wrt.setPosition(wrtp);
//            plane.setPosition(plnp);
//            telemetry.addData("操作模式:", sequence== DP_DropUpward.Sequence.AIM?"aim1":(sequence== DP_DropUpward.Sequence.RUN?"run":(sequence== DP_DropUpward.Sequence.RELEASE?"drop":"null")));
//            telemetry.addData("大臂伸长:",amlDrive.getCurrentPosition());
//            telemetry.addData("大臂位置:",armDrive.getCurrentPosition());
//            telemetry.addData("底盘速度:",speed);
//            telemetry.addData("手腕位置:",wrtp);
//            telemetry.addData("index:",index);
//
//            telemetry.update();
//            if(gamepad1.dpad_up||gamepad1.dpad_down||gamepad1.dpad_right||gamepad1.dpad_left){
//                bark_tank_drive_period();
//            }else{
//                bark_drive_period();
//            }
//        }
//    }
//    double arm_start_ang=0;//the angel of arm when starting
//    double wrist_target_ang=0;//the angel wrist need to reach
//    double wrist_range=180;//range
//    double wrist_target_pos=60;//the pos wrist need to reach
//    double arm_encoder_range=2400;
//    double wrist_start_pos=0.6;//wrist's pos at angel=0;
//    double arm_current_angel;
//    private void bark_wrist_balancer(){//design for servo range 180
//        arm_current_angel=(armDrive.getCurrentPosition()%100*100.0)*360/arm_encoder_range;//get the current angel according to the encoder
//        wrist_target_pos=wrist_start_pos+arm_current_angel/wrist_range+wrist_target_pos/wrist_range;
//        wrt.setPosition(1-wrist_target_pos);
//    }
//    private void logic_period() {
//        //IMPORTANT: READ ALL
//        XCYBoolean.bulkRead();
//        //time of this period
//        period_time_sec = time.seconds() - last_time_sec;
//        //P.S. telemetry comes from parent class, caption = title
////        telemetry.addData("elapse time", period_time_sec * 1000);
//        last_time_sec = time.seconds();
//        //send data
////        telemetry.update();
//        //clear read(ed) cache
//        for (LynxModule module : allHubs) {
//            module.clearBulkCache();
//        }
//    }
//    public void setArmLength(int length){
//        amlDrive.setTargetPosition(length);
//        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        amlDrive.setPower(1);
//    }
//    public void setArmPosition(int pos){
//        if(armDrive.getCurrentPosition()>pos){
//            armDrive.setPower(0.6);
//        }else{
//            armDrive.setPower(0.8);
//        }
//        armDrive.setTargetPosition(pos);
//        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armDrive.setPower(0.8);
//
//    }
//    public void bark_drive_period(){
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double x = -gamepad1.left_stick_x;
//        double y = gamepad1.left_stick_y;
//        double rx = (gamepad1.right_stick_x) * 0.7;
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) - Math.abs(rx), 1);
//        double frontLeftPower = ((rotY + rotX - rx)) / denominator;
//        double backLeftPower = ((rotY - rotX - rx)) / denominator;
//        double frontRightPower = ((rotY - rotX + rx)) / denominator;
//        double backRightPower = ((rotY + rotX + rx)) / denominator;
//        leftFrontDrive.setPower(frontLeftPower*speed);
//        leftBackDrive.setPower(backLeftPower*speed);
//        rightFrontDrive.setPower(frontRightPower*speed);
//        rightBackDrive.setPower(backRightPower*speed);
//    }
//    public void bark_tank_drive_period(){
//        double frontLeftPower = 0;
//        double backLeftPower = 0;
//        double frontRightPower = 0;
//        double backRightPower = 0;
//        if(gamepad1.dpad_up){
//            frontLeftPower = 1;
//            backLeftPower = 1;
//            frontRightPower = 1;
//            backRightPower = 1;
//        }
//        if(gamepad1.dpad_down){
//            frontLeftPower = -1;
//            backLeftPower = -1;
//            frontRightPower = -1;
//            backRightPower = -1;
//        }
//        if(gamepad1.dpad_right){
//            frontLeftPower = 1;
//            backLeftPower = -1;
//            frontRightPower = -1;
//            backRightPower = 1;
//        }
//        if(gamepad1.dpad_left){
//            frontLeftPower = -1;
//            backLeftPower = 1;
//            frontRightPower = 1;
//            backRightPower = -1;
//        }
//        leftFrontDrive.setPower(frontLeftPower*speed);
//        leftBackDrive.setPower(backLeftPower*speed);
//        rightFrontDrive.setPower(frontRightPower*speed);
//        rightBackDrive.setPower(backRightPower*speed);
//    }
//    public void sleep_with_drive(double time_mm) {
//        long start_time = System.currentTimeMillis();
//        while (opModeIsActive() && System.currentTimeMillis() - start_time < time_mm) {
//            bark_drive_period();
//        }
//    }
//}
