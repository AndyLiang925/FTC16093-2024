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
public class DPDrive16093_yjnSingle extends LinearOpMode {//
    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel\
    private DcMotorEx hangLeft   = null;
    private DcMotorEx hangRight   = null;
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
    public double driver_speed=1.0;//手动慢速档 driver controlled slow mode speed
    public double rotation_speed=1.0;//旋转降速 extra slow speed during rotation
    private double heading_target;//heading target
    private double grab1_open=0.6;
    private double grab1_close=0.86;
    private double grab2_open=0.23;
    private double grab2_close=0.5;
    private ElapsedTime runtime = new ElapsedTime();

    private BarkMecanumDrive bdrive;

    enum Sequence {
        AIM, RELEASE, RUN
    }
    int mode=0;
    private DPDrive16093_yjnSingle.Sequence sequence;
    //    NormalizedColorSensor colorSensor;
//    NormalizedColorSensor colorSensor2;
    View relativeLayout;
    @Override//
    public void runOpMode() throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        time = NanoClock.system();
        //get all Lynx Module device objects
        allHubs = hardwareMap.getAll(LynxModule.class);
//
//        relativeLayout.post(new Runnable() {
//            public void run() {
//                relativeLayout.setBackgroundColor(Color.WHITE);
//            }
//        });
//        float gain = 2;
//        final float[] hsvValues = new float[3];
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
//
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }
//        if (colorSensor2 instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor2).enableLight(true);
//        }

        //init position
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double wrtp = grab2_open;
        double plnp=0.6;//plane pos
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
        boolean leftGrabOpen=false;
        boolean rightGrabOpen=false;
        boolean colorSensorUsed=true;
        boolean leftColorRe=true;
        boolean rightColorRe=true;
        boolean PDSleep;
        boolean remove_limit=false;
        XCYBoolean aim =new XCYBoolean(()->gamepad1.x);
        XCYBoolean distal = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean proximal = new XCYBoolean(()->gamepad1.a);
        XCYBoolean drop = new XCYBoolean(()->gamepad1.left_bumper);
        XCYBoolean brake_start = new XCYBoolean(()->gamepad1.b);
        //XCYBoolean back = new XCYBoolean(()->gamepad2.x);
        XCYBoolean leftGrab = new XCYBoolean(()->gamepad1.right_trigger>0);
        XCYBoolean rightGrab = new XCYBoolean(()->gamepad1.left_trigger>0);
        XCYBoolean humanGrab = new XCYBoolean(()->gamepad1.back);
        XCYBoolean toRun = new XCYBoolean(()->gamepad1.left_stick_button);
        XCYBoolean armBack = new XCYBoolean(()->gamepad2.left_bumper);
        XCYBoolean armExpandBack = new XCYBoolean(()->gamepad2.right_bumper);
        XCYBoolean armDropUp = new XCYBoolean(()->gamepad2.dpad_up);
        XCYBoolean armDropDown = new XCYBoolean(()->gamepad2.dpad_down);
        XCYBoolean TankForward = new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean TankBackward = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean TankLeftward = new XCYBoolean(()->gamepad1.dpad_left);
        XCYBoolean TankRightward = new XCYBoolean(()->gamepad1.dpad_right);
        XCYBoolean hangLower = new XCYBoolean(()->gamepad2.a);
        XCYBoolean hangUp = new XCYBoolean(()->gamepad2.y);
        XCYBoolean plane_shoot = new XCYBoolean(()->gamepad1.y);
        XCYBoolean movePixel = new XCYBoolean(()->gamepad2.left_stick_button);
        XCYBoolean dpad = new XCYBoolean(()->gamepad1.dpad_left||gamepad1.dpad_up||gamepad1.dpad_down);
        //XCYBoolean slowMode = new XCYBoolean(()->gamepad1.right_trigger>0||gamepad1.left_trigger>0);
        XCYBoolean remove_endgame_limit = new XCYBoolean(()->gamepad1.right_trigger>0&&gamepad1.left_trigger>0);
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rearRight");
        hangLeft  = hardwareMap.get(DcMotorEx.class, motor_name_0);
        hangRight = hardwareMap.get(DcMotorEx.class, motor_name_1);
        armDrive  = hardwareMap.get(DcMotorEx.class, "arm");
        amlDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrt = hardwareMap.get(Servo.class, "wrist");
        gb1 = hardwareMap.get(Servo.class, "grab1");
        gb2 = hardwareMap.get(Servo.class, "grab2");
        brake = hardwareMap.get(Servo.class, "brake");
        plane = hardwareMap.get(Servo.class, "plane");
        imu = hardwareMap.get(IMU.class, "imu");

//        if(slowMode.get()){
//            speed=grab2_close;
//        }else{
//            speed=1;
//        }

        //手动慢速 driver-controlled slow mode
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
        amlDrive.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);
        wrt.setDirection(Servo.Direction.FORWARD);
        brake.setDirection(Servo.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sequence = Sequence.RUN;

        gb1.setPosition(grab1_close);
        gb2.setPosition(grab2_close);
        wrt.setPosition(0.9);
        plnp=0.6;
        plane.setPosition(plnp);
        brkp=0.5;
        brake.setPosition(brkp);
        heading_target=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        waitForStart();

        while (opModeIsActive()){
//            colorSensor.setGain(gain);
//            colorSensor2.setGain(gain);
//            NormalizedRGBA colors = colorSensor.getNormalizedColors();
//            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
//            Color.colorToHSV(colors.toColor(), hsvValues);
            logic_period();
            if(remove_endgame_limit.get()){
                remove_limit=true;
            }
            if(brake_start.toTrue()&&(runtime.seconds()>90||remove_limit)){
                if(brkp==0.5){
                    brkp=0.93;
                }else{
                    brkp=0.5;
                }
                //brkp=grab2_close1;
                brake.setPosition(brkp);
            }
//            if(slowMode.get()){
//                driver_speed=0.5;
//            }else{
//                driver_speed=1;
//            }
            if(armBack.toTrue()) {
                armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armDrive.setPower(backPower);
                sleep_with_drive(200);
                armDrive.setPower(0);
                armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(plane_shoot.get()&&(runtime.seconds()>90||remove_limit)){
                plnp=0.9;
            }
            if(armExpandBack.toTrue()) {
                amlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                amlDrive.setPower(backPower);
                sleep_with_drive(200);
                amlDrive.setPower(0);
                amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad1.back){
                imu.resetYaw();
            }
            if(toRun.toTrue()){
                if(sequence== Sequence.AIM){
                    setArmLength(0);
                }
                if(sequence== Sequence.RELEASE){
                    wrtp=0.9;
                    wrt.setPosition(wrtp);
                    speed=1;
                    setArmPosition(1200);
                    sleep_with_drive(500);
                }
                if(sequence== Sequence.AIM&&wrtp==0.45){
                    speed=1;
                    sleep_with_drive(500);
                }
                setArmPosition(0);
                setArmLength(0);
                if(sequence== Sequence.RELEASE){
                    speed=1;
                    sleep_with_drive(500);
                }
                wrtp=0.9;
                wrt.setPosition(wrtp);
                sequence= DPDrive16093_yjnSingle.Sequence.RUN;
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
                sequence= DPDrive16093_yjnSingle.Sequence.RELEASE;
                telemetry.addData("re lease",0);
                leftGrabOpen=false;
                rightGrabOpen=false;
                gb1.setPosition(leftGrabOpen?0.7:grab1_close);
                gb2.setPosition(rightGrabOpen?0.32:grab2_close);

            }
            if(aim.toTrue()){
                setArmLength(0);
                setArmPosition(0);
                wrtp=0.51;
                wrt.setPosition(wrtp);
                sequence= DPDrive16093_yjnSingle.Sequence.AIM;
                telemetry.addData("aim",0);
//                if(colorSensorUsed&&!grabbed(colors)){
//                    rightGrabOpen=true;
//                }
//                if(colorSensorUsed&&!grabbed(colors2)){
//                    leftGrabOpen=true;
//                }
                //need to change
                leftGrabOpen=false;
                rightGrabOpen=false;
                gb1.setPosition(leftGrabOpen?0.7:grab1_close);
                gb2.setPosition(rightGrabOpen?0.32:grab2_close);
            }

            if(humanGrab.toTrue()){
                colorSensorUsed=false;
            }

            if(sequence== DPDrive16093_yjnSingle.Sequence.AIM){
                speed = 0.5;
                if(distal.toTrue()){
                    setArmPosition(220);
                    sleep_with_drive(200);
                    setArmLength(570);
                    wrtp=0.45;
                    wrt.setPosition(wrtp);
                }
                if(proximal.toTrue()){
                    setArmLength(0);
                    wrtp=0.51;
                    wrt.setPosition(wrtp);
                    setArmPosition(0);
                }
//                    if(leftGrab.toTrue()){
//                        gb1.setPosition(leftGrabOpen?grab1_open:grab1_close);
//                        leftGrabOpen=!leftGrabOpen;
//                    }
//                    if(rightGrab.toTrue()){
//                        gb2.setPosition(rightGrabOpen?grab2_close:0.3);
//                        rightGrabOpen=!rightGrabOpen;
//                    }
//                if(colorSensorUsed){
//                    if (leftGrab.toTrue()) {
//                        leftGrabOpen = !leftGrabOpen;
//                    }
//                    if(!grabbed(colors2)){
//                        leftColorRe=true;
//                    }
//                    if(grabbed(colors2)&&leftColorRe){
//                        leftGrabOpen=false;
//                        leftColorRe=false;
//                    }
//                    gb1.setPosition(leftGrabOpen?grab1_open:grab1_close);
//                    if (rightGrab.toTrue()) {
//                        rightGrabOpen = !rightGrabOpen;
//                    }
//                    if(!grabbed(colors)){
//                        rightColorRe=true;
//                    }
//                    if(grabbed(colors)&&rightColorRe){
//                        rightGrabOpen=false;
//                        rightColorRe=false;
//                    }
//                    if(colorSensorUsed&&grabbed(colors)&&grabbed(colors2)){
//                        rightGrabOpen=false;
//                        leftGrabOpen=false;
//                        gb1.setPosition(leftGrabOpen?grab1_open:grab1_close);
//                        gb2.setPosition(rightGrabOpen?grab2_open:grab2_close);
//                        sleep_with_drive(300);
//                        setArmLength(0);
//                        setArmPosition(0);
//                        wrtp=0.9;
//                        wrt.setPosition(wrtp);
//                        sequence=DPDrive16093.Sequence.RUN;
//                        telemetry.addData("run",0);
//                    }
//                    gb2.setPosition(rightGrabOpen?grab2_open:grab2_close);
//                }else{
                if (leftGrab.toTrue()) {
                    leftGrabOpen = !leftGrabOpen;
                    gb1.setPosition(leftGrabOpen?grab1_open:grab1_close); //grab1_open
                }
                if (rightGrab.toTrue()) {
                    rightGrabOpen = !rightGrabOpen;
                    gb2.setPosition(rightGrabOpen?grab2_open:grab2_close); //grab2_open
                }
                //}
            }
            if(sequence== DPDrive16093_yjnSingle.Sequence.RUN){
                setArmLength(0);
                setArmPosition(0);
                wrtp=0.9;
                wrt.setPosition(wrtp);
                speed=1;
                gb1.setPosition(grab1_close);
                gb2.setPosition(grab2_close);
                if(distal.toTrue()){
                    setArmLength(0);
                    setArmPosition(0);
                    wrtp=0.51;
                    wrt.setPosition(wrtp);
                    sequence= DPDrive16093_yjnSingle.Sequence.AIM;
                    telemetry.addData("aim",0);
                    leftGrabOpen=false;
                    rightGrabOpen=false;
                    setArmPosition(220);
                    sleep_with_drive(200);
                    setArmLength(570);
                    wrtp=0.45;
                    wrt.setPosition(wrtp);
                    gb1.setPosition(leftGrabOpen?0.7:grab1_close);
                    gb2.setPosition(rightGrabOpen?0.32:grab2_close);
                }
                if(proximal.toTrue()){
                    setArmLength(0);
                    setArmPosition(0);
                    wrtp=0.51;
                    wrt.setPosition(wrtp);
                    sequence= DPDrive16093_yjnSingle.Sequence.AIM;
                    telemetry.addData("aim",0);
                    leftGrabOpen=false;
                    rightGrabOpen=false;
                    setArmLength(0);
                    wrtp=0.51;
                    wrt.setPosition(wrtp);
                    setArmPosition(0);
                    gb1.setPosition(leftGrabOpen?0.7:grab1_close);
                    gb2.setPosition(rightGrabOpen?0.32:grab2_close);
                }
            }
            if(mode==1&&armDrive.getCurrentPosition()>armPosLevels[index]-300){
                setArmLength(armLengthLevels[index]);
                mode=0;
            }
            if(sequence== DPDrive16093_yjnSingle.Sequence.RELEASE) {
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
                    gb1.setPosition(0.98);
                    gb2.setPosition(0.64);
                }
                wrt.setPosition(wrtp);
                if (leftGrab.toTrue()) {
                    rightGrabOpen = !rightGrabOpen;
                    gb1.setPosition(leftGrabOpen?0.7:grab1_close);
                    gb2.setPosition(rightGrabOpen?0.32:grab2_close);
                }
                if (rightGrab.toTrue()) {

                    leftGrabOpen = !leftGrabOpen;
                    gb1.setPosition(leftGrabOpen?0.7:grab1_close);
                    gb2.setPosition(rightGrabOpen?0.32:grab2_close);
                }

                ////
//                    if (leftGrab.toTrue()) {
//                        gb1.setPosition(leftGrabOpen?grab1_open:grab1_close);
//                        leftGrabOpen = !leftGrabOpen;
////                        setArmPosition(armPosLevels[index]-130);//
////                        sleep(300);
////                        setArmPosition(armPosLevels[index]);
//                    }
//                    if (rightGrab.toTrue()) {
//                        gb2.setPosition(rightGrabOpen?grab2_close:0.3);
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
            telemetry.addData("run_time :", runtime.seconds());
            telemetry.addData("imu_radian :",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("target_radian :",heading_target);
            telemetry.addData("操作模式:", sequence== DPDrive16093_yjnSingle.Sequence.AIM?"aim1":(sequence== DPDrive16093_yjnSingle.Sequence.RUN?"run":(sequence== DPDrive16093_yjnSingle.Sequence.RELEASE?"drop":"null")));//drive mode
            telemetry.addData("大臂伸长:",amlDrive.getCurrentPosition());//arm expand position
            telemetry.addData("大臂位置:",armDrive.getCurrentPosition());//arm position
            telemetry.addData("底盘速度:",speed);// robot speed
            telemetry.addData("手腕位置:",wrtp);//wrist position
            telemetry.addData("index:",index);//level of upper system
            //telemetry.addData("color_sensor抓没抓到:",grabbed(colors2)?"抓到了":"没抓到");//grabbed or not
//            telemetry.addLine()
//                    .addData("Red", "%.3f", colors2.red)
//                    .addData("Green", "%.3f", colors2.green)
//                    .addData("Blue", "%.3f", colors2.blue);
//            telemetry.addLine()
//                    .addData("Hue", "%.3f", hsvValues[0])
//                    .addData("Saturation", "%.3f", hsvValues[1])
//                    .addData("Value", "%.3f", hsvValues[2]);
//            telemetry.addData("Alpha", "%.3f", colors.alpha);

//            telemetry.addData("leftFront_velo",bdrive.getMotorVelo(1));
//            telemetry.addData("leftBack_velo",bdrive.getMotorVelo(2));
//            telemetry.addData("rightFront_velo",bdrive.getMotorVelo(3));
//            telemetry.addData("rightBack_velo",bdrive.getMotorVelo(4));

            telemetry.update();
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
//                }
//            });
            if(dpad.toTrue()){
                heading_target=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            if(gamepad1.dpad_up||gamepad1.dpad_down||gamepad1.dpad_right||gamepad1.dpad_left){
                bark_tank_drive_period(true,heading_target);
            }else{
                bark_drive_period();
            }
        }
    }
    public boolean grabbed(NormalizedRGBA c){
        //return (c.red>0.07&&c.green>0.15&&c.blue>0.15)||(c.red>0.1)||(c.green>0.1)||(c.blue>0.1);
        return (c.red>0.04&&c.green>0.10&&c.blue>0.10)||(c.red>0.05)||(c.green>0.05)||(c.blue>0.05);
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
        amlDrive.setTargetPosition(length);
        amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        amlDrive.setPower(0.85);
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
    public void bark_tank_drive_period(boolean use_heading_correction, double target_heading){
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rx = -(target_heading-botHeading)*3;
        if(gamepad1.right_stick_x!=0){
            rx=0;
            heading_target=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        if(gamepad1.dpad_up){
            frontLeftPower = 1-rx;
            backLeftPower = 1-rx;
            frontRightPower = 1+rx;
            backRightPower = 1+rx;
        }
        if(gamepad1.dpad_down){
            frontLeftPower = -1-rx;
            backLeftPower = -1-rx;
            frontRightPower = -1+rx;
            backRightPower = -1+rx;
        }
        if(gamepad1.dpad_right){
            frontLeftPower = 1-rx;
            backLeftPower = -1-rx;
            frontRightPower = -1+rx;
            backRightPower = 1+rx;
//            if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)>target_heading&&use_heading_correction){
//                frontRightPower += Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//                backRightPower -= Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//            }else if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)<target_heading&&use_heading_correction){
//                frontLeftPower -= Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//                backLeftPower += Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//            }
        }
        if(gamepad1.dpad_left){
            frontLeftPower = -1-rx;
            backLeftPower = 1-rx;
            frontRightPower = 1+rx;
            backRightPower = -1+rx;
//            if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)>target_heading&&use_heading_correction){
//                frontRightPower -= Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//                backRightPower += Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//            }else if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)<target_heading&&use_heading_correction){
//                frontLeftPower -= Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//                backLeftPower += Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-target_heading)/2;
//            }

        }
        double raw=gamepad1.right_stick_x;
        leftFrontDrive.setPower(frontLeftPower*speed*driver_speed*rotation_speed-raw*speed*driver_speed*rotation_speed);
        leftBackDrive.setPower(backLeftPower*speed*driver_speed*rotation_speed-raw*speed*driver_speed*rotation_speed);
        rightFrontDrive.setPower(frontRightPower*speed*driver_speed*rotation_speed+raw*speed*driver_speed*rotation_speed);
        rightBackDrive.setPower(backRightPower*speed*driver_speed*rotation_speed+raw*speed*driver_speed*rotation_speed);
    }
    public void sleep_with_drive(double time_mm) {
        long start_time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start_time < time_mm) {
            bark_drive_period();
        }
    }
}