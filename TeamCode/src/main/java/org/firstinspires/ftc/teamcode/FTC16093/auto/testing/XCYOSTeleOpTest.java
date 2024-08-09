package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTC16093.XCYBoolean;
import org.firstinspires.ftc.teamcode.FTC16093.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.NewSuperStructure;

import XCYOS.Task;
import XCYOS.TaskChainBuilder;
import XCYOS.XCYOSCore;

@TeleOp
public class XCYOSTeleOpTest extends LinearOpMode {
    State state;

    enum State {
        HOLDING_PIXEL,
        NOT_HOLDING_PIXEL
    }

    public static double x_static_compensation = 0.06;
    public static double y_static_compensation = 0.06;
    private static final double max_turn_assist_power = 0.4;
    private double global_drive_power = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        NewMecanumDrive drive = new NewMecanumDrive();
        NewSuperStructure upper = new NewSuperStructure();
        XCYOSCore.setUp(this, drive, upper);
        Task driveControl = new Task() {
            @Override
            public void run() {
                double x = -gamepad1.left_stick_y * 0.65 - gamepad1.right_stick_y * 0.3;
                double y = -gamepad1.left_stick_x * 0.65 - gamepad1.right_stick_x * 0.3;
                double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
                Vector2d correcting_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
                double corrected_rad = correcting_stick.angle() - drive.getPoseEstimate().getHeading();
                while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
                while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
                if (Math.abs(corrected_rad) < Math.PI / 5) {
                    double div = clamp(
                            Math.toDegrees(corrected_rad) / 20, 1)
                            * max_turn_assist_power * correcting_stick.norm();
                    turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
                }
                Pose2d power = (new Pose2d(x, y, turn_val*global_drive_power)).times(1);
                drive.setGlobalPower(power, x_static_compensation, y_static_compensation);
                XCYOSCore.getTelemetry().addData("power",power.vec().norm());
                XCYOSCore.log("drive control running");
            }
        };
        Task autoAim = new Task() {
            @Override
            public void run() {

            }
        };
        autoAim.setComponentsOccupied(drive, upper);
        autoAim.setPriority(2);
        driveControl.setComponentsOccupied(drive);
        driveControl.setPriority(-1);
        driveControl.setType(Task.Type.BASE);

        XCYOSCore.addTask(driveControl);

        XCYBoolean toOriginal = new XCYBoolean(() -> gamepad1.x && !gamepad1.back);
        XCYBoolean toBackDrop = new XCYBoolean(() -> gamepad1.y && !gamepad1.back);
        XCYBoolean resetSlider = new XCYBoolean(() -> gamepad1.y && gamepad1.back);
        XCYBoolean resetImu = new XCYBoolean(() -> gamepad1.x && gamepad1.back);
        XCYBoolean toFarIntake = new XCYBoolean(() -> state == State.NOT_HOLDING_PIXEL && gamepad1.dpad_up);
        XCYBoolean toCloseIntake = new XCYBoolean(() -> state == State.NOT_HOLDING_PIXEL && gamepad1.dpad_down);
        XCYBoolean higherBackDrop = new XCYBoolean(() -> state == State.HOLDING_PIXEL && gamepad1.dpad_up);
        XCYBoolean lowerBackDrop = new XCYBoolean(() -> state == State.HOLDING_PIXEL && gamepad1.dpad_down);
        XCYBoolean leftGrabber = new XCYBoolean(() -> gamepad1.left_bumper);
        XCYBoolean rightGrabber = new XCYBoolean(() -> gamepad1.right_bumper);
        XCYBoolean intakeBack = new XCYBoolean(() -> gamepad1.b);
        boolean leftGrabbed = false;
        boolean rightGrabbed = false;
        int backDropLength = 0;
        Vector2d lastGrabPos = new Vector2d();
        state = State.NOT_HOLDING_PIXEL;
        waitForStart();

        XCYOSCore.addTask(upper.toOriginal());
        while (opModeIsActive()) {
            if (resetImu.toTrue()){
                XCYOSCore.addTask(drive.resetImu(new Pose2d(0, 0, Math.toRadians(180))));
            }
            if (higherBackDrop.toTrue()) {
                backDropLength = Range.clip(backDropLength + 70, 0, NewSuperStructure.MAX_SLIDE_LENGTH);
                upper.setSlide(backDropLength, 0.8);
            }
            if (lowerBackDrop.toTrue()) {
                backDropLength = Range.clip(backDropLength - 70, 0, NewSuperStructure.MAX_SLIDE_LENGTH);
                upper.setSlide(backDropLength, 0.8);
            }
            if (toOriginal.toTrue()) {
                state = State.NOT_HOLDING_PIXEL;
                global_drive_power = 1;
                XCYOSCore.addTask(upper.toOriginal());
            }
            if (resetSlider.toTrue()) {
                XCYOSCore.addTask(upper.resetSliderEnc());
            }
            if (toFarIntake.toTrue()) {
                state = State.NOT_HOLDING_PIXEL;
                global_drive_power=0.3;
                leftGrabbed=rightGrabbed=false;
                XCYOSCore.addTask(upper.toIntakeGround());
            }
            if (toCloseIntake.toTrue()){
                state = State.NOT_HOLDING_PIXEL;
                leftGrabbed=rightGrabbed=false;
                XCYOSCore.addTask(new TaskChainBuilder(1,upper)
                        .add(()->{
                            upper.setArmPosition(0);
                            upper.setSlide(0,1);
                            upper.setWristPosition(NewSuperStructure.wrt_intake_min_pos);
                            upper.openGrabber(NewSuperStructure.Grabber.ALL);
                        }).end().getBase());
            }
            if (leftGrabber.toTrue()) {
                if (state == State.NOT_HOLDING_PIXEL) {
                    if (leftGrabbed) upper.openGrabber(NewSuperStructure.Grabber.LEFT);
                    else upper.closeGrabber(NewSuperStructure.Grabber.LEFT);
                    leftGrabbed = !leftGrabbed;
                } else
                    upper.releaseGrabber(NewSuperStructure.Grabber.LEFT);
            }
            if (rightGrabber.toTrue()) {
                if (state == State.NOT_HOLDING_PIXEL) {
                    if (rightGrabbed) upper.openGrabber(NewSuperStructure.Grabber.RIGHT);
                    else upper.closeGrabber(NewSuperStructure.Grabber.RIGHT);
                    rightGrabbed = !rightGrabbed;
                } else
                    upper.releaseGrabber(NewSuperStructure.Grabber.RIGHT);
            }
            if (intakeBack.toTrue()) {
                global_drive_power=1;
                state = State.HOLDING_PIXEL;
                if (upper.grabberIsClosed(NewSuperStructure.Grabber.LEFT) && upper.grabberIsClosed(NewSuperStructure.Grabber.RIGHT))
                    XCYOSCore.addTask(upper.intakeBack());
                else
                    XCYOSCore.addTask(new TaskChainBuilder(1, upper)
                            .add(() -> upper.closeGrabber(NewSuperStructure.Grabber.ALL))
                            .sleep(300)
                            .add(upper.intakeBack())
                            .end().getBase());
            }
            if (toBackDrop.toTrue()) {
                global_drive_power=0.3;
                state = State.HOLDING_PIXEL;
                XCYOSCore.addTask(upper.toBackDrop(backDropLength));
            }
            XCYOSCore.log(state.toString());
            XCYOSCore.update();
            XCYBoolean.bulkRead();
        }
    }

    public static double clamp(double val, double limit) {
        return Range.clip(val, -limit, limit);
    }
}
