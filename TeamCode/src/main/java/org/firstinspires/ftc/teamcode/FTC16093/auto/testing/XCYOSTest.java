package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

import XCYOS.TaskChainBuilder;
import XCYOS.XCYOSCore;

@Autonomous
@Config
public class XCYOSTest extends LinearOpMode {
    public static double x = 12;
    public static double y = 0, heading = 0;
    public static int time = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);
        BarkMecanumDrive drive = new BarkMecanumDrive(hardwareMap);
        BarkMecanumDrive mecanumDrive = new BarkMecanumDrive(hardwareMap);
        TaskChainBuilder tb = new TaskChainBuilder();
        tb.add(drive.simpleMoveTime(new Pose2d(x, y, Math.toRadians(heading)), time, 0.8, 1))
                .add(drive.simpleMoveTime(new Pose2d(0, 0, Math.toRadians(0)), time, 0.8, 1))
                .end();
        XCYOSCore.addTask(tb.getBase());
        XCYOSCore.addTask(mecanumDrive.updatePositionTask);

        XCYOSCore.setUp(this);
        waitForStart();
        while (opModeIsActive()) {
            XCYOSCore.update();
        }

    }
}
