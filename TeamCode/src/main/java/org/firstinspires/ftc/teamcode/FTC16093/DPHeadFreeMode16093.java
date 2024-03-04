//package org.firstinspires.ftc.teamcode.FTC16093;
//
//import android.app.Activity;
//import android.graphics.Color;
//import android.view.View;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import java.util.Vector;
//import java.util.Locale;
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.hardware.SwitchableLight;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//
//import org.firstinspires.ftc.robotcore.external.Func;
//
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.internal.system.Deadline;
//
//import java.util.concurrent.TimeUnit;
//import android.app.Activity;
//import android.graphics.Color;
//import android.view.View;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.SwitchableLight;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//@TeleOp
//@Disabled
//public class DPHeadFreeMode16093 extends LinearOpMode{
//    NormalizedColorSensor colorSensor;//color
//    NormalizedColorSensor colorSensor1;//color1
//    View relativeLayout;//color
//    View relativeLayout1;//color1
//    private Servo aml = null;
//    private Servo gb1 = null;
//    private Servo gb2 = null;
//    private Servo wrt = null;
//    private Servo fja = null;
//    private DcMotor leftFrontDrive   = null;
//    private DcMotor rightFrontDrive  = null;
//    private DcMotor leftBackDrive    = null;
//    private DcMotor rightBackDrive   = null;
//    private DcMotor armDrive   = null;
//    public static String motor_name_0 = "servoMotor";
//    //LED 灯带
//    private final static int LED_PERIOD = 10;
//
//    /*
//     * Rate limit gamepad button presses to every 500ms.
//     */
//    private final static int GAMEPAD_LOCKOUT = 500;
//
//    RevBlinkinLedDriver blinkinLedDriver;
//    RevBlinkinLedDriver.BlinkinPattern pattern;
//
//    Telemetry.Item patternName;
//    Telemetry.Item display;
//    LEDTest.DisplayKind displayKind;
//    Deadline ledCycleDeadline;
//    Deadline gamepadRateLimit;
//
//    protected enum DisplayKind {
//        MANUAL,
//        AUTO
//    }
//
//
//    /*
//     * handleGamepad
//     *
//     * Responds to a gamepad button press.  Demonstrates rate limiting for
//     * button presses.  If loop() is called every 10ms and and you don't rate
//     * limit, then any given button press may register as multiple button presses,
//     * which in this application is problematic.
//     *
//     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
//     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
//     */
//    protected void handleGamepad()
//    {
//        if (!gamepadRateLimit.hasExpired()) {
//            return;
//        }
//        //pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
//        displayPattern();
//        gamepadRateLimit.reset();
//    }
//
//    protected void setDisplayKind(LEDTest.DisplayKind displayKind)
//    {
//        this.displayKind = displayKind;
//        display.setValue(displayKind.toString());
//    }
//
//    protected void doAutoDisplay()
//    {
//        if (ledCycleDeadline.hasExpired()) {
//            pattern = pattern.next();
//            displayPattern();
//            ledCycleDeadline.reset();
//        }
//    }
//
//    protected void displayPattern()
//    {
//        blinkinLedDriver.setPattern(pattern);
//        patternName.setValue(pattern.toString());
//    }
//    //灯带结束
//    TouchSensor touchSensor;
//    int is_zhuan=0;
//    int drive_mode=0;
//    IMU imu;
//    Orientation angles;
//    Acceleration gravity;
//    @Override public void runOpMode(){
//        DcMotorSimple motor0 = hardwareMap.get(DcMotorSimple.class, motor_name_0);
//        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
//        aml = hardwareMap.get(Servo.class, "liumo");
//        gb1 = hardwareMap.get(Servo.class, "gb1");
//        gb2 = hardwareMap.get(Servo.class, "gb2");
//        wrt = hardwareMap.get(Servo.class, "wrt");
//        fja = hardwareMap.get(Servo.class, "aaa");
//        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
//
//        aml.setDirection(Servo.Direction.FORWARD);
//        gb1.setDirection(Servo.Direction.REVERSE);
//        gb2.setDirection(Servo.Direction.REVERSE);
//        wrt.setDirection(Servo.Direction.FORWARD);
//        //fja.setDirection(Servo.Direction.REVERSE);
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        //parameter box
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//        RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//
//
//        // Wait until we're told to go
//
//        // Start the logging of measured acceleration
//        imu.resetYaw();
//        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "fl");
//        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "fr");
//        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "rl");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rr");
//        armDrive = hardwareMap.get(DcMotorEx.class, "arm");
//        //LED
//        displayKind = LEDTest.DisplayKind.AUTO;
//
//        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
//        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//        blinkinLedDriver.setPattern(pattern);
//
//        display = telemetry.addData("Display Kind: ", displayKind.toString());
//        patternName = telemetry.addData("Pattern: ", pattern.toString());
//
//        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
//        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
//        //
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        armDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        double speed = 1;
//        double fjpos = 0.58;
//        double wrtpos = 0.57;
//        double gb1p = 0.09;
//        double gb2p = 0.95;
//        double amlp = 0.1;
//        int armp=20;
//        int turn_mode=0;
//        //color sensor
//
//        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//        int relativeLayoutId1 = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout1", "id", hardwareMap.appContext.getPackageName());
//        relativeLayout1 = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId1);
//        float gain = 2;
//
//        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
//        // hue, the second element (1) will contain the saturation, and the third element (2) will
//        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
//        // for an explanation of HSV color.
//        final float[] hsvValues = new float[3];
//        final float[] hsvValues1 = new float[3];
//
//        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
//        // state of the X button on the gamepad
//        boolean xButtonPreviouslyPressed = false;
//        boolean xButtonCurrentlyPressed = false;
//
//        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
//        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
//        // the values you get from ColorSensor are dependent on the specific sensor you're using.
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }
//        if (colorSensor1 instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor1).enableLight(true);
//        }
//        //
//        fja.setPosition(fjpos);
//        gb1.setPosition(gb1p);
//        gb2.setPosition(gb2p);
//        wrt.setPosition(wrtpos);
//        aml.setPosition(amlp);
//        waitForStart();
//
//        while (opModeIsActive()){
//            handleGamepad();//LED
//            //color sensor
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.WHITE);
//                }
//            });
////            relativeLayout1.post(new Runnable() {
////                public void run() {
////                    relativeLayout1.setBackgroundColor(Color.WHITE);
////                }
////            });
//            telemetry.addData("Gain", gain);
//            colorSensor.setGain(gain);
//            colorSensor1.setGain(gain);
//            NormalizedRGBA colors = colorSensor.getNormalizedColors();
//            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
//            Color.colorToHSV(colors.toColor(), hsvValues);
//            Color.colorToHSV(colors1.toColor(), hsvValues1);
//            if(gamepad2.dpad_down){
//                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
//            }else if(colors.red>0.01&&colors.green>0.01&&colors.blue>0.01&&colors1.red>0.01&&colors1.green>0.01&&colors1.blue>0.01){
//                telemetry.addData("一号爪子:", "抓到了");
//                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//            }else if(colors.red>0.01&&colors.green>0.01&&colors.blue>0.01) {
//                telemetry.addData("一号爪子:", "没抓到");
//                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
//            }else if(colors1.red>0.01&&colors1.green>0.01&&colors1.blue>0.01){
//                telemetry.addData("二号爪子:", "抓到了");
//                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
//            }else{
//                telemetry.addData("二号爪子:", "没抓到");
//                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
//            }
//            telemetry.addLine()
//                    .addData("Red", "%.3f", colors.red)
//                    .addData("Green", "%.3f", colors.green)
//                    .addData("Blue", "%.3f", colors.blue);
//            telemetry.addLine()
//                    .addData("Red", "%.3f", colors1.red)
//                    .addData("Green", "%.3f", colors1.green)
//                    .addData("Blue", "%.3f", colors1.blue);
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
//
//                }
//            });
////            relativeLayout1.post(new Runnable() {
////                public void run() {
////                    relativeLayout1.setBackgroundColor(Color.HSVToColor(hsvValues1));
////
////                }
////            });
//            //
//            if(touchSensor.isPressed()) {
//                armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            //纸飞机
//            if(gamepad1.x){
//                fjpos = 0.35;
//
//            }
//            if(fjpos<0.58){
//                fjpos+=0.01;
//            }
//            if(gamepad1.b){
//            }
//            //悬挂
//            if(gamepad1.left_bumper){
//                motor0.setPower(-1);
//            }else if(gamepad1.right_bumper){
//                motor0.setPower(1);
//            }else{
//                motor0.setPower(0);
//            }
//
//            //移动
//            if(gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0||turn_mode==1||turn_mode==5){
//                speed = 0.3;
//            }else{
//                speed = 1;
//            }
//
//
//            //无头
//            if(gamepad1.a) imu.resetYaw();
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double x = -gamepad1.left_stick_x;
//            double y = gamepad1.left_stick_y;
//            double rx = (gamepad1.right_stick_x) * 0.7;
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) - Math.abs(rx), 1);
//            double frontLeftPower = ((rotY + rotX - rx)) / denominator * speed;
//            double backLeftPower = ((rotY - rotX - rx)) / denominator * speed;
//            double frontRightPower = ((rotY - rotX + rx)) / denominator * speed;
//            double backRightPower = ((rotY + rotX + rx)) / denominator * speed;
//            //有头
//            //一键摆正
//            if(gamepad1.y){
//                while(!(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-90)<2)){
//                    double tspd=Math.max(0.3,Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-90)/150);//turn speed
//                    if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)>90){
//                        frontLeftPower = -tspd * speed;
//                        backLeftPower = -tspd * speed;
//                        frontRightPower = tspd * speed;
//                        backRightPower = tspd * speed;
//                        leftFrontDrive.setPower(frontLeftPower);
//                        leftBackDrive.setPower(backLeftPower);
//                        rightFrontDrive.setPower(frontRightPower);
//                        rightBackDrive.setPower(backRightPower);
//                    }else if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)<90){
//                        frontLeftPower = tspd * speed;
//                        backLeftPower = tspd * speed;
//                        frontRightPower = -tspd * speed;
//                        backRightPower = -tspd * speed;
//                        leftFrontDrive.setPower(frontLeftPower);
//                        leftBackDrive.setPower(backLeftPower);
//                        rightFrontDrive.setPower(frontRightPower);
//                        rightBackDrive.setPower(backRightPower);
//                    }
//                }
//            }
//                if(gamepad1.dpad_up){
//                    frontLeftPower = 0.6 * speed;
//                    backLeftPower = 0.6 * speed;
//                    frontRightPower = 0.6 * speed;
//                    backRightPower = 0.6 * speed;
//                }else if(gamepad1.dpad_down){
//                    frontLeftPower = -0.6 * speed;
//                    backLeftPower = -0.6 * speed;
//                    frontRightPower = -0.6 * speed;
//                    backRightPower = -0.6 * speed;
//                }else if(gamepad1.dpad_left){
//                    frontLeftPower = 0.6 * speed;
//                    backLeftPower = -0.6 * speed;
//                    frontRightPower = -0.6 * speed;
//                    backRightPower = 0.6 * speed;
//                }else if(gamepad1.dpad_right){
//                    frontLeftPower = -0.6 * speed;
//                    backLeftPower = 0.6 * speed;
//                    frontRightPower = 0.6 * speed;
//                    backRightPower = -0.6 * speed;
//                }
//
//            //防唐
//            if(armDrive.getCurrentPosition()<850&&armDrive.getCurrentPosition()>700&&amlp<0.2) {
//                gb1p=0.09;
//                gb1.setPosition(gb1p);
//                gb2p=0.95;
//                gb2.setPosition(gb2p);
//            }
//            //大臂转动
//            //boolean flag1 = false;
//            if(armDrive.getCurrentPosition()<200&&amlp<0.2&&wrtpos>0.32) {
//                gb1p=0.09;
//                gb1.setPosition(gb1p);
//                gb2p=0.95;
//                gb2.setPosition(gb2p);
//            }
//            if(gamepad2.y) {
//                armp=2430;
//                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armDrive.setPower(1);
//                gb1p=0.09;
//                gb1.setPosition(gb1p);
//                gb2p=0.95;
//                gb2.setPosition(gb2p);
//                wrtpos=0.6;
//                turn_mode=1;
//                amlp=0;
//                //while (opModeIsActive() && armDrive.isBusy()){
//                  //  telemetry.addData("大臂编码器位置",  "Running at %7d", armDrive.getCurrentPosition());
//                    //telemetry.update();
//
//                //}
//                //armDrive.setPower(0);
////                armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }else if(gamepad2.a) {
//                armp=0;
//                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armDrive.setPower(-1);
//                gb1p=0.09;
//                gb1.setPosition(gb1p);
//                gb2p=0.95;
//                gb2.setPosition(gb2p);
//                amlp=0;
//                wrtpos=0.5;
//                turn_mode=2;
////                while (opModeIsActive() && armDrive.isBusy()){
////                    telemetry.addData("大臂编码器位置",  "Running at %7d", armDrive.getCurrentPosition());
////                    telemetry.update();
////                }
//                //armDrive.setPower(0);
//                //armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            }else if(false&&gamepad2.x){
//                //大臂伸长一键到位
//                armp=2430;
//                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armDrive.setPower(1);
//                gb1p=0.09;
//                gb1.setPosition(gb1p);
//                gb2p=0.95;
//                gb2.setPosition(gb2p);
//                wrtpos=0.7;
//                turn_mode=4;
//                amlp=0;
//            }else if(gamepad2.right_stick_y<0){
//                turn_mode=0;
//                armp=armp+15;
//            }else if(gamepad2.right_stick_y>0){
//                armp=Math.max(armp-15,-1000);
//                turn_mode=0;
//            }
//            if(turn_mode==2&&armDrive.getCurrentPosition()<850&&armDrive.getCurrentPosition()>700){
//                wrtpos=0.5;
//            }
//            if(turn_mode==1&&armDrive.getCurrentPosition()>2350){
//                amlp=0.7;
//                turn_mode=5;
//            }
//            //大臂抓取伸出一键到位
//            if(gamepad2.x){
//                turn_mode=5;
//                armp=175;
//                amlp=0.7;
//                wrtpos=0.1;
//                gb1p=0.5;
//                gb1.setPosition(gb1p);
//                gb2p=0.5;
//                gb2.setPosition(gb2p);
//            }
//
//            //wrist
//            if (gamepad2.left_stick_y>0) {
//                wrtpos=wrtpos+(gamepad2.left_stick_y/50)>0.72?0.72:wrtpos+(gamepad2.left_stick_y/50);
//            }else if(gamepad2.left_stick_y<0){
//                wrtpos=wrtpos+(gamepad2.left_stick_y/50)<0.09?0.09:wrtpos+(gamepad2.left_stick_y/50);
//            }
//
//            //grab
//            if (gamepad2.left_bumper) {
//                gb1p=0.09;
//            }else if(gamepad2.left_trigger>0){
//                gb1p=0.5;
//            }
//            if (gamepad2.right_bumper) {
//                gb2p=0.95;
//            }else if(gamepad2.right_trigger>0) {
//                gb2p = 0.5;
//            }
//            //一键到地位抓取
//            if (gamepad2.dpad_up) {
//                armp=0;
//                wrtpos=0.14;
//                amlp=0.2;
//                //amlp=amlp+0.02>0.7?0.7:amlp+0.02;
//            }else if(gamepad2.dpad_down){
//                //amlp=amlp-0.02<0?0:amlp-0.02;
//            }
//
//            //设置+显示
//            armDrive.setTargetPosition(armp);
//            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            if(armDrive.getCurrentPosition()<armp){
//                armDrive.setPower(-1);
//            }else{
//                armDrive.setPower(1);
//            }
//
//            leftFrontDrive.setPower(frontLeftPower);
//            leftBackDrive.setPower(backLeftPower);
//            rightFrontDrive.setPower(frontRightPower);
//            rightBackDrive.setPower(backRightPower);
//            fja.setPosition(fjpos);
//            gb1.setPosition(gb1p);
//            gb2.setPosition(gb2p);
//            wrt.setPosition(wrtpos);
//            aml.setPosition(amlp);
//
//            if(touchSensor.isPressed()){
//                telemetry.addData("is pressed",1);
//            }else{
//                telemetry.addData("not pressed",1);
//            }
//            telemetry.addData("armDrive pos", armDrive.getCurrentPosition());
//            telemetry.addData("leftFrontDrive", frontLeftPower);
//            telemetry.addData("\nleftBackDrive", backLeftPower);
//            telemetry.addData("\nrightFrontDrive", frontRightPower);
//            telemetry.addData("\nrightBackDrive", backRightPower);
//            telemetry.addData("\nyaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("\nfjpos=", fjpos);
//            telemetry.addData("\nx=", x);
//            telemetry.addData("\ny=", y);
//            telemetry.addData("\nrx=", rx);
//            telemetry.addData("\nwrtp=", wrtpos);
//            telemetry.update();
//        }
//
//    }
//}