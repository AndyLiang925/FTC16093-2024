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
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

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
    public static boolean using_armExternalEnc = false;
    public static boolean set_power_mode_or_set_position_mode = false;
    public static String arm_encoder = "arm";
    private Servo wrist = null;

    public static double wristPos = 1;
    public SuperStructure upper;

    @Override
    public void runOpMode() {
        upper = new SuperStructure(this);
        wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, arm_encoder);
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "armExpand");
        DcMotorEx armExternalEnc = hardwareMap.get(DcMotorEx.class,"hangRight");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        
        if (reset) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        if (reverse_arm) {
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(reverse_slide){
            slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while (opModeIsActive()) {
            if (!read_only) {
                if (using_armExternalEnc){
                    upper.setArmPosition(armPos);
                }else{
                    armMotor.setTargetPosition(armPos);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(max_power);
                }
                sleep(2000);
                slideMotor.setTargetPosition(slidePos);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(max_power);
                wrist.setPosition(wristPos);
            }
            else{
                wrist.setPosition(wristPos);
            }
            telemetry_M.addData("is busy_arm", armMotor.isBusy());
            telemetry_M.addData("is busy_slide", slideMotor.isBusy());

            if (using_armExternalEnc){
                telemetry_M.addData("armExternalEncoder",armExternalEnc.getCurrentPosition());
            }else{
                telemetry_M.addData("encoder_arm", armMotor.getCurrentPosition());
            }
            telemetry_M.addData("encoder_slide", slideMotor.getCurrentPosition());
            telemetry_M.addData("velocity_arm", armMotor.getVelocity());
            telemetry_M.addData("velocity_slide", slideMotor.getVelocity());
            telemetry_M.update();
        }
    }
}
