package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;

@Autonomous
@Config
public class XCYOSTest extends LinearOpMode {
    public static double x=0;
    public static double y=0,heading=0;
    public static int time=500;
    @Override
    public void runOpMode() throws InterruptedException {
        BarkMecanumDrive drive = new BarkMecanumDrive(hardwareMap);
        waitForStart();
        drive.initSimpleMove(new Pose2d(x, y, Math.toRadians(heading)));
        while (drive.isBusy())
            drive.update();
        long endTime = System.currentTimeMillis()+time;
        while (endTime>System.currentTimeMillis())
            drive.update();

    }
}
