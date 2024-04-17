//package org.firstinspires.ftc.teamcode.FTC16093.test;//package org.firstinspires.ftc.teamcode.FTC16093;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
////Assets from Qualcomm
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//
//@TeleOp(name = "intake test")
//@Config
//public class IntakeTest extends LinearOpMode {
//
//    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//    public static int encoder_position = 1150;
//    public static int encoder_position_1 = 1150;
//    public static double wrist_pos = 0.5;
//    public static double max_power = 1;
//    public static boolean read_only = false;
//    public static boolean reverse_0 = false;
//    public static boolean reverse_1 = true;
//    public static boolean reset = true;
//    public static boolean set_power_mode_or_set_position_mode = false;
//    public static String motor_name_0 = "arm";
//    public static String motor_name_1 = "armExpand";
//    public static String servo_name_0 = "wrist";
//
//    @Override
//    public void runOpMode() {
//        DcMotorEx motor0 = hardwareMap.get(DcMotorEx.class, motor_name_0);
//        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, motor_name_1);
//        Servo servo0 =  hardwareMap.get(Servo.class, servo_name_0);
//        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        waitForStart();
//        if (reset) {
//            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        if (reverse_0) {
//            motor0.setDirection(DcMotorSimple.Direction.REVERSE);
////            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//        else if(reverse_1){
//            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//
//        while (opModeIsActive()) {
//            if (set_power_mode_or_set_position_mode) {
//                if (read_only) {
//                    motor0.setPower(0);
//                    motor1.setPower(0);
//                }
//                else {
//                    motor0.setPower(max_power);
//                    motor1.setPower(max_power);
//                }
//                motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//            } else {
//                if (!read_only) {
//                    servo0.setPosition(wrist_pos);
//                    motor0.setTargetPosition(encoder_position);
//                    motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motor0.setPower(max_power);
//                    motor1.setTargetPosition(encoder_position_1);
//                    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motor1.setPower(max_power);
//                    sleep(10000);
//                    motor0.setTargetPosition(0);
//                    motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motor0.setPower(max_power);
//                    motor1.setTargetPosition(0);
//                    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motor1.setPower(max_power);
//                }
//                telemetry_M.addData("is busy_1", motor0.isBusy());
////                telemetry_M.addData("encoder_1", motor0.getCurrentPosition());
////                telemetry_M.addData("is busy_3", motor1.isBusy());
////                telemetry_M.addData("encoder_2", motor1.getCurrentPosition());
//            }
////            telemetry_M.addData("current_0",motor0.getCurrent(CurrentUnit.MILLIAMPS));
////            telemetry_M.addData("current_1",motor1.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry_M.addData("encoder_0", motor0.getCurrentPosition());
////            telemetry_M.addData("encoder_1", motor1.getCurrentPosition());
//            telemetry_M.addData("velocity_1", motor0.getVelocity());
////            telemetry_M.addData("velocity_2", motor1.getVelocity());
//            telemetry_M.update();
//        }
//    }
//}
