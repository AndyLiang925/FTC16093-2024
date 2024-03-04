//package org.firstinspires.ftc.teamcode.FTC16093;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//
//@TeleOp(name = "servo test")
//@Config
//public class ServoTest extends LinearOpMode {
//
//    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//    public static boolean read_only = false;
//    public static double servo_pos1 = 0.5;
////    public static double servo_pos2 = 0.49;
//    public static String servo_name1 = "armR";
////    public static String servo_name2 = "armL";
//    private Servo servo0=null;
////    private Servo servo1=null;
//
//    @Override
//    public void runOpMode() {
//
//        servo0 = hardwareMap.get(Servo.class, servo_name1);
//  //      servo0.setDirection(Servo.Direction.REVERSE);
////        servo1 = hardwareMap.get(Servo.class, servo_name2);
////        servo1 = hardwareMap.get(Servo.class,"servo1");
//
//        waitForStart();
//        while (opModeIsActive()) {
//            if (!read_only) {
//                servo0.setPosition(servo_pos1);
////                servo1.setPosition(servo_pos2);
//                telemetry_M.addData("leftFront", servo0.getPosition());
////                telemetry_M.addData("rightfront", servo1.getPosition());
//
//
//                telemetry_M.update();
////
//            }
//        }
//    }
//}
