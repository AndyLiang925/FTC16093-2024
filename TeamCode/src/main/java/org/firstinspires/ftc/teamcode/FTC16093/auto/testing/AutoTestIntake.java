package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

import XCYOS.Task;
import XCYOS.TaskChainBuilder;
import XCYOS.XCYOSCore;

@Autonomous
public class AutoTestIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);
        BarkMecanumDrive drive = new BarkMecanumDrive(hardwareMap);
        BarkMecanumDrive mecanumDrive = new BarkMecanumDrive(hardwareMap);
        TaskChainBuilder tb = new TaskChainBuilder();
        tb.add(drive.simpleMoveTime(new Pose2d(0,1.5,0),2000))
                .end();
        XCYOSCore.addTask(tb.getBase());
        XCYOSCore.addTask(mecanumDrive.updatePositionTask);

        XCYOSCore.setUp(this);
        waitForStart();
        while (opModeIsActive()){
            XCYOSCore.update();
        }
    }

}
