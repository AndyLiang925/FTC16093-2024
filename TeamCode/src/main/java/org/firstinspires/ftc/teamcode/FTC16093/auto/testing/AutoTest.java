package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.NewSuperStructure;

import XCYOS.Component;
import XCYOS.TaskChainBuilder;
import XCYOS.XCYOSCore;

@Autonomous
@Config
public class AutoTest extends LinearOpMode {
    Pose2d startPos;

    public static double startPos_x = 12.125, startPos_y = 59, startPos_heading = 90;

    public static double x = 12;
    public static double y = 0, heading = 0;
    public static int time = 2000;

    public static Pose2d spikeMark_redLeft = new Pose2d(23, -45, Math.toRadians(140));
    public static Pose2d spikeMark_redCenter = new Pose2d(22, -46, Math.toRadians(105));
    public static Pose2d spikeMark_redRight = new Pose2d(21, -54, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        NewSuperStructure upper = new NewSuperStructure( );
        NewMecanumDrive drive = new NewMecanumDrive( );
        NewMecanumDrive mecanumDrive = new NewMecanumDrive( );

        startPos = new Pose2d(startPos_x, startPos_y * (-1), Math.toRadians(startPos_heading * (-1)));

        TaskChainBuilder tb = new TaskChainBuilder();
        tb.add(drive.simpleMoveTime(startPos, time, 0.8, 1))
                .add(drive.simpleMoveTime(spikeMark_redRight, time, 0.8, 1))
                .add(upper.toPlaceMarker())
                .end();
        XCYOSCore.addTask(tb.getBase());
        XCYOSCore.addTask(mecanumDrive.updatePositionTask);
        XCYOSCore.addTask(mecanumDrive.updatePositionTask);
        XCYOSCore.setUp(this);
        waitForStart();

        while (opModeIsActive()) {
            XCYOSCore.update();
        }

    }
}
