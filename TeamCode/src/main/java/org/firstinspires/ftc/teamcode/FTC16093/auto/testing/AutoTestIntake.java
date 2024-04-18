package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

import XCYOS.Task;
import XCYOS.TaskChainBuilder;
import XCYOS.XCYOSCore;

@Autonomous
public class AutoTestIntake extends AutoMaster {

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this, () -> {
        });
        BarkMecanumDrive mecanumDrive = new BarkMecanumDrive(hardwareMap);
        XCYOSCore.setTelemetry(telemetry);
        TaskChainBuilder tb = new TaskChainBuilder();
        tb.add(new Task() {
                    @Override
                    public void run() {
                        superstructure.setArmPosition(100);
                        if (superstructure.getArmPosition() > 90) {
                            status = Status.ENDED;
                        }
                    }
                })
                .add(new Task() {
                    @Override
                    public void run() {
                        superstructure.grab1_open();
                        status = Status.ENDED;
                    }
                });
        XCYOSCore.addTask(tb.getBase());
        XCYOSCore.addTask(mecanumDrive.updatePositionTask);
    }

}
