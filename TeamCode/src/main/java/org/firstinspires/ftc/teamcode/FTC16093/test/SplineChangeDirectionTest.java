package org.firstinspires.ftc.teamcode.FTC16093.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous
@Config
public class SplineChangeDirectionTest extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int ang1,ang2,ang3;
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory untitled0 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(ang1)))
                .splineToSplineHeading(new Pose2d(50, 0, Math.toRadians(ang3)), Math.toRadians(ang2))
                .build();

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineToSplineHeading(new Pose2d(20,20,Math.toRadians(0)),Math.toRadians(0))
//                .build();

        drive.followTrajectory(untitled0);
        telemetry_M.update();
        sleep(200000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );


    }
}