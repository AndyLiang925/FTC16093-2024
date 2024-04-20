package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC16093.XCYBoolean;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

import XCYOS.XCYOSCore;

@TeleOp
@Config
public class TestArm extends LinearOpMode {
    public static int position = 1000;
    public static double x = 12;
    public static double y = 0, heading = 0;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);
        BarkMecanumDrive drive =new BarkMecanumDrive(hardwareMap);
        XCYBoolean testMove = new XCYBoolean(()->gamepad1.b);
        waitForStart();
        Runnable update = ()->{drive.update();superstructure.update();XCYBoolean.bulkRead();};
        while (opModeIsActive()) {
            if (gamepad1.a) {
                superstructure.setArmPosition(position);
            } else {
                superstructure.setArmPosition(0);
            }
            if (testMove.toTrue()){
                drive.initSimpleMove(new Pose2d(x, y, Math.toRadians(heading)));
            }
            if (testMove.toFalse()){
                drive.stopTrajectory();
                drive.setMotorPowers(0,0,0,0);
            }
            telemetry_M.addData("enc", superstructure.getArmPosition());
            telemetry_M.update();
            update.run();
        }
    }

}
