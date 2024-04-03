package org.firstinspires.ftc.teamcode.FTC16093.test;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous
public class SplineChangeDirectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory untitled0 = drive.trajectoryBuilder(new Pose2d(24.34, 0.79, Math.toRadians(181.49)))
                .splineToSplineHeading(new Pose2d(-42.68, -0.79, Math.toRadians(1)), Math.toRadians(-5.83))
                .build();



        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(20,20,Math.toRadians(0)),Math.toRadians(0))
                .build();

        drive.followTrajectory(untitled0);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );


    }
}