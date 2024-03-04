//package org.firstinspires.ftc.teamcode.FTC16093;
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
//import org.firstinspires.ftc.teamcode.util.Encoder;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
///*
// * This is a simple routine to test turning capabilities.
// */
//@TeleOp
//public class EncoderTest16093 extends LinearOpMode {
//    public static double ANGLE = 90; // deg
//    private DcMotorEx rightFrontDrive  = null;
//    private DcMotorEx rightBackDrive  = null;
//    private DcMotorEx leftBackDrive  = null;
//    private Encoder leftEncoder, rightEncoder, frontEncoder;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //rightFrontDrive = hardwareMap.get(DcMotorEx.class, "fl");
//        //rightBackDrive = hardwareMap.get(DcMotorEx.class, "rl");
//        //leftBackDrive = hardwareMap.get(DcMotorEx.class, "fr");
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rl"));
////        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        //rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        waitForStart();
//        while(opModeIsActive()){
////            telemetry.addData("Encoder: ", rightFrontDrive.getCurrentPosition());
////            telemetry.addData("Encoder: ", rightBackDrive.getCurrentPosition());
////            telemetry.addData("Encoder: ", leftBackDrive.getCurrentPosition());
//            telemetry.addData("Encoder: ", leftEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//}
