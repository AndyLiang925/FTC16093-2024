package org.firstinspires.ftc.teamcode.FTC16093.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
@Config
public class ArmLevelTest3in1 extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int armPos = 1850;
    public static int slidePos = 0;
    public static double max_power = 1;
    public static boolean read_only = false;
    public static boolean reverse_arm = false;
    public static boolean reverse_slide = true;
    public static boolean reset = true;
    public static boolean set_power_mode_or_set_position_mode = false;
    private Servo wrist = null;

    public static double wristPos = 1;

    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "armExpand");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        
        if (reset) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        if (reverse_arm) {
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(reverse_slide){
            slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while (opModeIsActive()) {
            
            if (set_power_mode_or_set_position_mode) {
                if (read_only) {
                    armMotor.setPower(0);
                    slideMotor.setPower(0);
                }
                else {
                    armMotor.setPower(max_power);
                    slideMotor.setPower(max_power);
                }
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            } else {
                if (!read_only) {
                    armMotor.setTargetPosition(armPos);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(2000);
                    armMotor.setPower(max_power);
                    slideMotor.setTargetPosition(slidePos);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotor.setPower(max_power);
                    wrist.setPosition(wristPos);

                    sleep(10000);
                    armMotor.setTargetPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(max_power);
                    slideMotor.setTargetPosition(0);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotor.setPower(max_power);
                }
                else{
                    wrist.setPosition(wristPos);
                }
                telemetry_M.addData("is busy_arm", armMotor.isBusy());
                telemetry_M.addData("is busy_slide", slideMotor.isBusy());
            }

            telemetry_M.addData("encoder_arm", armMotor.getCurrentPosition());
            telemetry_M.addData("encoder_slide", slideMotor.getCurrentPosition());
            telemetry_M.addData("velocity_arm", armMotor.getVelocity());
            telemetry_M.addData("velocity_slide", slideMotor.getVelocity());
            telemetry_M.update();
        }
    }
}
