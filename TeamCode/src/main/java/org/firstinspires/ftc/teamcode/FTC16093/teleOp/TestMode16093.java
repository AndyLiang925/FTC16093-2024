package org.firstinspires.ftc.teamcode.FTC16093.teleOp;//package org.firstinspires.ftc.teamcode.FTC16093;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.FTC16093.XCYBoolean;

@TeleOp
public class TestMode16093 extends LinearOpMode {
    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel\
    private DcMotorEx hangLeft   = null;
    private DcMotorEx hangRight   = null;
    private DcMotorEx armDrive   = null;
    private DcMotorEx amlDrive   = null;
    private Servo gb1 = null;
    private Servo gb2 = null;
    private Servo wrt = null;
    enum Sequence {
        AIM, RELEASE
    }

    @Override public void runOpMode(){
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double wrtp = 0;
        int amlp=0;
        int armp=0;
        XCYBoolean aim =new XCYBoolean(()->gamepad1.a||gamepad2.a);
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rearRight");
        //hangLeft  = hardwareMap.get(DcMotorEx.class, "hangLeft");
        //hangRight = hardwareMap.get(DcMotorEx.class, "hangRight");
        armDrive  = hardwareMap.get(DcMotorEx.class, "arm");
        amlDrive = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrt = hardwareMap.get(Servo.class, "wrist");
        gb1 = hardwareMap.get(Servo.class, "grab2");
        gb2 = hardwareMap.get(Servo.class, "grab1");
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        //hangLeft.setDirection(DcMotorEx.Direction.FORWARD);
        //hangRight.setDirection(DcMotorEx.Direction.REVERSE);
        amlDrive.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);
        wrt.setDirection(Servo.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //hangLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //hangRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        amlDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){

            drive  = -gamepad1.left_stick_y  / 3.5;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 3.5;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 4.5;  // Reduce turn rate to 33%.
            if (gamepad2.left_stick_y>0) {
                wrtp=wrtp+(gamepad2.left_stick_y/70)>1?1:wrtp+(gamepad2.left_stick_y/70);
            }else if(gamepad2.left_stick_y<0){
                wrtp=wrtp+(gamepad2.left_stick_y/70)<0?0:wrtp+(gamepad2.left_stick_y/70);
            }
            if (gamepad2.left_trigger>0) {
                gb1.setPosition(0.22);
            }else if(gamepad2.left_bumper){
                gb1.setPosition(0.53);
            }
            if (gamepad2.right_trigger>0) {
                gb2.setPosition(0.76);
            }else if(gamepad2.right_bumper){
                gb2.setPosition(0.45);
            }
            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            //hangLeft.setPower(gamepad1.right_bumper?-1:(gamepad1.left_bumper?1:0));
            //hangRight.setPower(gamepad1.right_bumper?-1:(gamepad1.left_bumper?1:0));
            //armDrive.setPower(gamepad1.a?-0.6:(gamepad1.y?0.6:0));
            //amlDrive.setPower(gamepad1.dpad_up?-0.6:(gamepad1.dpad_down?0.6:0));
            armp+=gamepad2.right_stick_y*15;
            if(gamepad2.y){
                amlp+=15;
            }else if(gamepad2.a){
                amlp-=15;
            }
            if(gamepad2.b){

            }bark_wrist_balancer();
            armDrive.setTargetPosition(armp);
            amlDrive.setTargetPosition(amlp);
            amlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(amlDrive.getCurrentPosition()<amlp){
                amlDrive.setPower(-1);
            }else{
                amlDrive.setPower(1);
            }
            if(armDrive.getCurrentPosition()<armp){
                armDrive.setPower(-1);
            }else{
                armDrive.setPower(1);
            }
            //wrt.setPosition(wrtp);
            moveRobot(drive, strafe, turn);
            sleep(10);
            telemetry.addData("wrist pos:",wrtp);
            telemetry.addData("arm pos:",armp);
            telemetry.addData("arm expand pos:",amlp);
            telemetry.update();
        }
    }
    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    double arm_start_ang=-50;//the angle of arm when starting
    double wrist_target_ang=120;//the angle wrist need to reach
    double wrist_range=180;//range
    double wrist_current_pos=60;//the pos
    //double wrist_target_ang=60;//the pos wrist need to reach
    double arm_encoder_range=4000;
    double wrist_start_pos=0.34;//wrist's pos at angle=0
    double arm_current_angle;
    private void bark_wrist_balancer(){//design for servo range 180
        arm_current_angle=armDrive.getCurrentPosition()*360/arm_encoder_range+arm_start_ang;//get the current angle according to the encoder
        wrist_current_pos=wrist_start_pos-arm_current_angle/180+wrist_target_ang/360;//
        wrt.setPosition(wrist_current_pos);
        telemetry.addData("arm_angle:", arm_current_angle);
        telemetry.addData("wrist:", wrist_current_pos);
    }
}
