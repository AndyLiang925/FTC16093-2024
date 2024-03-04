//package org.firstinspires.ftc.teamcode.backup;
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
//
//@Disabled
//public class OpenDrive16093 extends LinearOpMode {
//    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
//    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
//    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
//    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel\
//    private DcMotorEx hangLeft   = null;
//    private DcMotorEx hangRight   = null;
//    private DcMotorEx armDrive   = null;
//    private DcMotorEx amlDrive   = null;
//    private Servo gb1 = null;
//    private Servo gb2 = null;
//    private Servo wrt = null;
//    private IMU imu;
//    enum Sequence {
//        AIM, RELEASE
//    }
//
//    @Override public void runOpMode(){
//        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
//        double wrtp = 0;
//        int amlp=0;
//        int armp=0;
//        int index=0;
//        int armLengthLevels[] = {1,2,3};
//        int armPosLevels[] = {1,2,3};
//        double wrtLevels[] = {0.1,0.2,0.3};
//        boolean leftGrabOpen=false;
//        boolean rightGrabOpen=false;
//        boolean colorSensorUsed=true;
//        Sequence sequence = Sequence.AIM;
//        XCYBoolean aim =new XCYBoolean(()->gamepad1.a||gamepad2.a);
//        XCYBoolean distal = new XCYBoolean(()->gamepad2.dpad_up);
//        XCYBoolean proximal = new XCYBoolean(()->gamepad2.dpad_down);
//        XCYBoolean drop = new XCYBoolean(()->gamepad2.y);
//        XCYBoolean back = new XCYBoolean(()->gamepad2.x);
//        XCYBoolean leftGrab = new XCYBoolean(()->gamepad2.left_trigger>0);
//        XCYBoolean rightGrab = new XCYBoolean(()->gamepad2.right_trigger>0);
//        XCYBoolean humanGrab = new XCYBoolean(()->gamepad2.back);
//        XCYBoolean slow = new XCYBoolean(()->gamepad1.right_trigger>0||gamepad1.left_trigger>0);
//        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "fl");
//        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "fr");
//        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "rl");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rr");
//        hangLeft  = hardwareMap.get(DcMotorEx.class, "hl");
//        hangRight = hardwareMap.get(DcMotorEx.class, "hr");
//        armDrive  = hardwareMap.get(DcMotorEx.class, "arm");
//        amlDrive = hardwareMap.get(DcMotorEx.class, "aml");
//        wrt = hardwareMap.get(Servo.class, "wrt");
//        gb1 = hardwareMap.get(Servo.class, "gb1");
//        gb2 = hardwareMap.get(Servo.class, "gb2");
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        //parameter box
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        hangLeft.setDirection(DcMotorEx.Direction.FORWARD);
//        hangRight.setDirection(DcMotorEx.Direction.REVERSE);
//        amlDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        armDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        wrt.setDirection(Servo.Direction.REVERSE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        hangLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        hangRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        waitForStart();
//        while (opModeIsActive()){
//            if(colorSensorUsed){
//                telemetry.addData("color sensor",0);
//            }else{
//                if(humanGrab.toTrue()){
//                    colorSensorUsed=false;
//                }
//                if(sequence==Sequence.AIM){
//                    if(distal.toTrue()){
//                        setArmLength(1000);
//                        wrt.setPosition(0.5);
//                        setArmPosition(300);
//                    }
//                    if(proximal.toTrue()){
//                        setArmLength(0);
//                        wrt.setPosition(0.6);
//                        setArmPosition(0);
//                    }
//                    if(drop.toTrue()){
//                        setArmLength();
//                        setArmPosition();
//                        wrt.setPosition();
//                        sequence=Sequence.RELEASE;
//                    }
//                    if(leftGrab.toTrue()){
//                        gb1.setPosition(leftGrabOpen?0.6:0.5);
//                        leftGrabOpen=!leftGrabOpen;
//                    }
//                    if(rightGrab.toTrue()){
//                        gb2.setPosition(rightGrabOpen?0.4:0.5);
//                        rightGrabOpen=!rightGrabOpen;
//                    }
//                }
//                if(sequence==Sequence.RELEASE){
//                    if(distal.toTrue()){
//                        index++;
//                    }
//                    if(proximal.toTrue()){
//                        index--;
//                    }
//                    setArmLength(armLengthLevels[index]);
//                    setArmPosition(armPosLevels[index]);
//                    wrt.setPosition(wrtLevels[index]);
//                    if(leftGrab.toTrue()){
//                        gb1.setPosition(leftGrabOpen?0.6:0.5);
//                        leftGrabOpen=!leftGrabOpen;
//                    }
//                    if(rightGrab.toTrue()){
//                        gb2.setPosition(rightGrabOpen?0.4:0.5);
//                        rightGrabOpen=!rightGrabOpen;
//                    }
//                    if(back.toTrue()){
//                        setArmLength();
//                        setArmPosition();
//                        wrt.setPosition();
//                        sequence=Sequence.AIM;
//                    }
//                }
//            }
//
//            drive_period();
//        }
//    }
//    public void setArmLength(int length){
//        amlDrive.setTargetPosition(length);
//        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        amlDrive.setPower(1);
//    }
//    public void setArmPosition(int pos){
//        armDrive.setTargetPosition(pos);
//        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armDrive.setPower(1);
//
//    }
//    public void drive_period(){
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
//}
