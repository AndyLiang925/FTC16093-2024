package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

@TeleOp
@Config
public class TestArm extends LinearOpMode {
    public static int position = 1000;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                superstructure.setArmPosition(position);
            } else {
                superstructure.setArmPosition(0);
            }
            telemetry_M.addData("enc", superstructure.getArmPosition());
            telemetry_M.update();
            superstructure.update();
        }
    }

}
