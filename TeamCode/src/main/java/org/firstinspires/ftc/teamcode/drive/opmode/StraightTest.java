package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    private Runnable update;
    private  Pose2d startPos = new Pose2d(0,0,Math.toRadians(0));
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SuperStructure upper = new SuperStructure(this);
        BarkMecanumDrive drive = new BarkMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);
        update = () -> {
            drive.update();
            upper.update();
        };
        upper.setUpdateRunnable(update);
        drive.setUpdateRunnable(update);


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

//        drive.followTrajectory(trajectory);
        drive.setSimpleMovePower(0.95);
        drive.moveTo(new Pose2d(50,0,Math.toRadians(0)),200);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
