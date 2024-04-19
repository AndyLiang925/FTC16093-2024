package org.firstinspires.ftc.teamcode.FTC16093.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTC16093.CenterStageVisionProcessor;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.SuperStructure;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AutoMaster extends LinearOpMode {


    private ElapsedTime runtime;

    public static final int PROXIMAL = 1;
    public static final int DISTAL = -1;
    public static final int LEFT = 0;
    public static final int RIGHT = 1;
    public static final int grab_blue = 1;
    public static final int grab_right_yellow = 2;
    public static boolean DEBUG = true;
    public static int startTimeDelay = -1, cycleDelay = -1;
    public static double DESIRED_DISTANCE = 0.4;

    public static final int RED = -1;
    public static final int BLUE = 1;

    protected int startSide;
    protected int side_color;
    protected int drop_side;
    public static int grab_side;
    protected boolean back_start = false;
    private boolean targetFound;

    private CenterStageVisionProcessor processor;
    private BarkMecanumDrive drive;

    private VisionPortal visionPortal;
    private VisionPortal visionPortalAprilTag;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    private int DesiredTagId = -1;

    private CenterStageVisionProcessor.StartingPosition startingPos;

    final double SPEED_GAIN = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN = 0.01;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE = 0.5;
    final double MAX_AUTO_TURN = 0.3;

    Pose2d startPos;
    Pose2d farPos;

    public static double startPos_x = 12.125, startPos_y = 59, startPos_heading = 90;
    public static double spikeMark_x, spikeMark_y, spikeMark_heading = 0;
    private static final double spikeMark_heading_near = Math.toRadians(180);

    Pose2d spikeMark = new Pose2d(spikeMark_x, spikeMark_y, Math.toRadians(spikeMark_heading));

    public static double spikeMarkMidYAxis = -47;
    public static double spikeMarkSide_x;

    public static Pose2d spikeMark_redLeft = new Pose2d(22, -45, Math.toRadians(145));
    public static Pose2d spikeMark_redCenter = new Pose2d(22, -46, Math.toRadians(105));
    public static Pose2d spikeMark_redRight = new Pose2d(21.7, -54, Math.toRadians(90));

    public static Pose2d spikeMark_blueLeft = new Pose2d(23.5, 51, Math.toRadians(-90)); //TODO
    public static Pose2d spikeMark_blueCenter = new Pose2d(22, 46, Math.toRadians(-95));
    public static Pose2d spikeMark_blueRight = new Pose2d(22, 45, Math.toRadians(-135));

    public static Pose2d spikeMark_red_distalLeft = new Pose2d(-48.2, -51, Math.toRadians(90));
    public static Pose2d spikeMark_red_distalCenter = new Pose2d(-46, -46, Math.toRadians(85));
    public static Pose2d spikeMark_red_distalRight = new Pose2d(-46, -45, Math.toRadians(45));

    public static Pose2d spikeMark_blue_distalLeft = new Pose2d(-46, 45, Math.toRadians(-40));
    public static Pose2d spikeMark_blue_distalCenter = new Pose2d(-46, 46, Math.toRadians(-85));
    public static Pose2d spikeMark_blue_distalRight = new Pose2d(-48.3, 54, Math.toRadians(-90));
    Pose2d yellowBackDropPosition = new Pose2d(48.3, 33 * side_color, Math.toRadians(180)); //47.8 blue
    public static int backDrop_heading = 180;
    public static double backDrop_centerAxis_y = 33.4;
    public static double pixel_width = 3.11;
    public static double drop_pos_index = 0;
    public static double drop_y;

    private Pose2d getDropPose(double pos) {
        return new Pose2d(47.9, side_color * backDrop_centerAxis_y + pos * pixel_width, Math.toRadians(backDrop_heading));
    } // distal 47.9

    protected Pose2d preDropPose;
    private Pose2d getPreDropPose(double pos){
        return new Pose2d(38, side_color * backDrop_centerAxis_y + pos * pixel_width, Math.toRadians(backDrop_heading));
    }
    private Pose2d ecDropPosition;
    public static int drop_pos_ec_index = 0;
    public static double intake_far_grab_x = -43, intake_farCenter_y = 9;
    public static double intake_side_y = 35;
    public static double intake_blue_right_oblique_x = -43, intake_blue_right_oblique_y = 32, intake_oblique_heading_grab1 = 180;
    public static double park_x = 43, park_inside = 56, park_outside = 6;
    public static double intake_nearGrab_x = -58.5, intake_distal_y = 9.6, intake_medi_side_x = -53, intake_medi_side_y = 40;
    private Pose2d intake_near;
    public static double intake_distal_sideNear_y = 32;
    public static double edgePass_y = 53;
    public static double intake_medi_x = -38; // intial -35
    public static double intake_medi_y = 6.5;


    public static boolean kickProp = false;
    public SuperStructure upper;
    public static int armPos_near1 = 269; //132
    public static int wait_time = 0;

    private Runnable update;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
    }

    protected void initHardware() throws InterruptedException {
        telemetry.addLine("init: webcam");
        telemetry.update();
        processor = new CenterStageVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), processor);
        telemetry.update();
        runtime = new ElapsedTime();
        runtime.reset();
        telemetry.addLine("init: drive");
        if (startSide == PROXIMAL) {
            startPos = new Pose2d(startPos_x * startSide, startPos_y * side_color, Math.toRadians(startPos_heading * side_color));
        } else {
            startPos = new Pose2d((startPos_x + 23.6) * startSide, startPos_y * side_color, Math.toRadians(startPos_heading * side_color));
        }
        upper = new SuperStructure(this);
        drive = new BarkMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(startPos);
        drive.update();
        drive.getLocalizer().setPoseEstimate(startPos);
        drive.update();
        telemetry.addLine("init: superstructure");
        upper.setGrabSide(side_color);
//        upper.reset_arm_encoders();
        update = () -> {
            drive.update();
            upper.update();
        };
        upper.setUpdateRunnable(update);
        drive.setUpdateRunnable(update);

        setUpAuto();

        telemetry.addLine("init: trajectory");

        while (!opModeIsActive()) {
            startingPos = processor.getStartingPosition();
            telemetry.addData("Vision", startingPos);
            telemetry.update();
            long time = System.currentTimeMillis();
            //telemetry.update();
            delay(15);
            //while (System.currentTimeMillis() - time < 100 && opModeInInit()) idle();
            if (isStopRequested()) throw new InterruptedException();
        }
        intake_near = new Pose2d(intake_nearGrab_x, intake_distal_y * side_color, Math.toRadians(180));
        if (startSide == PROXIMAL) {
            if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE) {
                DesiredTagId = 1;
                spikeMark = spikeMark_blueLeft;
                if (drop_side == LEFT) {
                    drop_pos_index = 2;
                } else {
                    drop_pos_index = 1;
                }
                drop_pos_ec_index = 0;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark = spikeMark_blueCenter;
                if (drop_side == LEFT) {
                    drop_pos_index = -0.5;
                } else {
                    drop_pos_index = -1;
                }
                drop_pos_ec_index = -2;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark = spikeMark_blueRight;
                if (drop_side == LEFT) {
                    drop_pos_index = -2;
                } else {
                    drop_pos_index = -3;
                }
                drop_pos_ec_index = 0;

            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                // index-yellowPosition:
                // 1: center_sideLeft 2:left_sideRight 3: left_sideLeft
                // 0: center_sideRight-1: right_sideRight -2: right_sideRight
                // index-extraPosition:
                // 0: center_sideLeft 1: right_sideLeft 2: right_side Right
                // -1:
                DesiredTagId = 4;
                spikeMark = spikeMark_redLeft;
                if (drop_side == LEFT) {
                    drop_pos_index = 3.2;
                } else {
                    drop_pos_index = 2.8;
                }
                drop_pos_ec_index = 0;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark = spikeMark_redCenter;
                if (drop_side == LEFT) {
                    drop_pos_index = 1;
                } else {
                    drop_pos_index = 0.8;
                }
                drop_pos_ec_index = 2;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark = spikeMark_redRight;
                if (drop_side == LEFT) {
                    drop_pos_index = -1.5;
                } else {
                    drop_pos_index = -2;
                }
                drop_pos_ec_index = 2;
            }
            yellowBackDropPosition = getDropPose(drop_pos_index);
            preDropPose = getPreDropPose(drop_pos_index);
            ecDropPosition = getDropPose(drop_pos_ec_index);
        }
        if (startSide == DISTAL) {
            if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE) {
                DesiredTagId = 1;
                spikeMark = spikeMark_blue_distalLeft;
                if (drop_side == LEFT) {
                    drop_pos_index = 1.8;
                } else {
                    drop_pos_index = 1;
                }
                drop_pos_ec_index = 0;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark = spikeMark_blue_distalCenter;
                if (drop_side == LEFT) {
                    drop_pos_index = -0.2;
                } else {
                    drop_pos_index = -1;
                }
                drop_pos_ec_index = -1;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark = spikeMark_blue_distalRight;
                if (drop_side == LEFT) {
                    drop_pos_index = -2.5;
                } else {
                    drop_pos_index = -3;
                }
                drop_pos_ec_index = 0;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark = spikeMark_red_distalLeft;
                if (drop_side == LEFT) {
                    drop_pos_index = 2.8;
                } else {
                    drop_pos_index = 2.8;
                }
                drop_pos_ec_index = 0;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark = spikeMark_red_distalCenter;
                if (drop_side == LEFT) {
                    drop_pos_index = 1;
                } else {
                    drop_pos_index = 0.8;
                }
                drop_pos_ec_index = 2;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark = spikeMark_red_distalRight;
                if (drop_side == LEFT) {
                    drop_pos_index = -1.4;
                } else {
                    drop_pos_index = -1.5;
                }
                drop_pos_ec_index = 1;
            }
            yellowBackDropPosition = getDropPose(drop_pos_index);
            preDropPose = getPreDropPose(drop_pos_index);
            ecDropPosition = getDropPose(drop_pos_ec_index);
        }
        runtime.reset();
        visionPortal.stopStreaming();
    }

    public void ApriltagDetection() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortalAprilTag = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DesiredTagId < 0) || (detection.id == DesiredTagId)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            }
        }
    }

    public void moveToSpikeMark() {
//        if (isStopRequested()) return;
//        TrajectorySequence moveToSpikeMark;
//        moveToSpikeMark = drive.trajectorySequenceBuilder(startPos)
//                .lineToLinearHeading(spikeMark)
//                .build();
//        drive.followTrajectorySequence(moveToSpikeMark);
        drive.moveTo(spikeMark, 200);
    }

    public void moveToBackDrop() {
        if (isStopRequested()) return;
        upper.grabPurple(side_color);
        upper.setArmPosition(3300);
        //drive.followTrajectory(moveToDetectedBackDrop);
        drive.moveTo(preDropPose, 400);
        upper.setArmPosition(4200);
        //drive.followTrajectory(moveToDetectedBackDrop_closer);
        drive.setSimpleMovePower(0.2);
        drive.moveTo(yellowBackDropPosition, 200);
    }

    public void intakeDistal() {
        if (isStopRequested()) return;
        drive.moveTo(new Pose2d(-57.8, intake_medi_side_y * side_color, Math.toRadians(180)), 0);
        drive.moveTo(new Pose2d(-57.8, intake_distal_y * side_color, Math.toRadians(180)), 0);
        drive.moveTo(new Pose2d(-54, intake_distal_y * side_color, Math.toRadians(180)), 0);
        upper.autoGrabPrepare(armPos_near1);
        drive.moveTo(new Pose2d(-54, intake_distal_y * side_color, Math.toRadians(180)), 300);

        drive.setSimpleMovePower(0.35);
        drive.setSimpleMoveTolerance(2,5);
        drive.moveTo(intake_near, 500);
        upper.autoGrabFinish();
        drive.setSimpleMovePower(0.8);
    }

    public void distalMoveToBackDrop() {
        drive.setSimpleMoveTolerance(1,0.1);
        drive.moveTo(new Pose2d(25,7*side_color,Math.toRadians(180)),0);
        upper.setArmPosition(3500);
        drive.setSimpleMovePower(0.5);
        drive.moveTo(preDropPose,100);
        drive.setSimpleMovePower(0.2);
        upper.setArmPosition(4200);
        drive.moveTo(yellowBackDropPosition, 500);
    }

    public void distal_backDropAxis() {
        drive.setSimpleMoveTolerance(1,0.1);
        drive.moveTo(new Pose2d(25,7*side_color,Math.toRadians(180)),0);
        upper.setArmPosition(3800);
        drive.setSimpleMovePower(0.5);
        drive.moveTo(preDropPose,100);
        drive.setSimpleMovePower(0.2);
        upper.setArmPosition(4200);
        drive.moveTo(new Pose2d(47, backDrop_centerAxis_y * side_color, Math.toRadians(180)), 500);
    }

    public void ec_lowFar_edgeSpline_blue() {
        TrajectorySequence moveToIntake = drive.trajectorySequenceBuilder(yellowBackDropPosition)
                .splineTo(new Vector2d(30, 54 * side_color), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-37, 54 * side_color, Math.toRadians(180.00)))
                //.waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(intake_blue_right_oblique_x, intake_blue_right_oblique_y, Math.toRadians(180)))
                .build();
        TrajectorySequence moveToDrop = drive.trajectorySequenceBuilder(new Pose2d(intake_blue_right_oblique_x, intake_blue_right_oblique_y, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-35, 53 * side_color, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40, 53 * side_color))
                .addDisplacementMarker(80, () -> upper.wrist_to_upward())
                .addDisplacementMarker(80, () -> upper.setArmPosition(1800))
                .lineToLinearHeading(ecDropPosition)
                .build();
        drive.followTrajectorySequence(moveToIntake);
        upper.intake2_lowFar();
        drive.followTrajectorySequence(moveToDrop);
    }

    public void ec_lowFar_edgeSpline_red() {
        drive.moveTo(new Pose2d(30, 54 * side_color, Math.toRadians(180.00)), 100);
    }

    /**
     * TODO moveTo
     *
     * @param side side==1: inside         side==2: outside
     */
    public void parking(int side) {
        if (side == 3) {
            delay(1500);
            return;
        }
        if (isStopRequested()) return;
        Trajectory moveToPark = drive.trajectoryBuilder(yellowBackDropPosition)
                .lineToLinearHeading(new Pose2d(park_x, park_inside * side_color, Math.toRadians(-90 * side_color)))
                .build();
        Trajectory moveToBack = drive.trajectoryBuilder(new Pose2d(park_x, park_inside * side_color, Math.toRadians(-90 * side_color)))
                .lineToConstantHeading(new Vector2d(park_x + 15, park_inside * side_color))
                .build();
        if (side == 2) {
            moveToPark = drive.trajectoryBuilder(yellowBackDropPosition)
                    .lineToLinearHeading(new Pose2d(park_x, park_outside * side_color, Math.toRadians(-90 * side_color)))
                    .build();
            moveToBack = drive.trajectoryBuilder(new Pose2d(park_x, park_outside * side_color, Math.toRadians(-90 * side_color)))
                    .lineToConstantHeading(new Vector2d(park_x + 15, park_outside * side_color))
                    .build();
        }
        drive.followTrajectory(moveToPark);
        drive.followTrajectory(moveToBack);
    }

    public void setUpAuto() {
        upper.setArmPosition(0);
        upper.setArmLength(-5, 1);
        upper.grab1_close();//gb1.setPosition(0.22);
        upper.grab2_close();//gb2.setPosition(0.45);
        upper.wrist_to_middle();
    }

    public void backDrop_move() {
        drive.moveTo(ecDropPosition, 500);
    }
    public void backDrop_move1Pixel() {
        Trajectory moveToAnotherDrop = drive.trajectoryBuilder(yellowBackDropPosition)
                .lineToLinearHeading(new Pose2d(47, backDrop_centerAxis_y + pixel_width, Math.toRadians(180)))
                .build();
        drive.followTrajectory(moveToAnotherDrop);
    }

    public void intakeGate() {
        TrajectorySequence moveToPixel = drive.trajectorySequenceBuilder(yellowBackDropPosition)
                .splineTo(new Vector2d(30, 6.5 * side_color), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(intake_medi_x, intake_medi_y * side_color, Math.toRadians(180)))
                .build();

        Trajectory intake = drive.trajectoryBuilder(new Pose2d(intake_medi_x, intake_medi_y * side_color, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(intake_far_grab_x, intake_farCenter_y*side_color, Math.toRadians(180)))
                .build();

        drive.followTrajectorySequence(moveToPixel);
        drive.followTrajectory(intake);
        upper.intake2_lowFar();

        drive.setSimpleMovePower(0.95);
        drive.moveTo(new Pose2d(30,intake_farCenter_y*side_color,Math.toRadians(180)),0);
        upper.setArmPosition(3500);
        upper.wrist_to_upward();
        drive.moveTo(preDropPose,100);
        upper.setArmPosition(4516);
        drive.setSimpleMoveTolerance(2,0.1);
        drive.setSimpleMovePower(0.2);
        drive.moveTo(ecDropPosition,100);
        //drive.followTrajectory(drop);//
    }

    public void distal_intakeSide_sideBack() {
        if (isStopRequested()) return;
        TrajectorySequence toMediByLine = drive.trajectorySequenceBuilder(spikeMark)
                .lineToLinearHeading(new Pose2d(intake_medi_side_x, intake_medi_side_y, Math.toRadians(180)))
                .build();
        Trajectory moveToCenter = drive.trajectoryBuilder(new Pose2d(intake_medi_side_x, intake_medi_side_y, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_medi_side_x, intake_distal_sideNear_y))
                .build();
        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(intake_medi_side_x, intake_distal_sideNear_y, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_nearGrab_x, intake_distal_sideNear_y))
                .build();
        Trajectory strafe = drive.trajectoryBuilder(new Pose2d(intake_nearGrab_x, intake_distal_sideNear_y, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_nearGrab_x + 2, edgePass_y * side_color))
                .build();
        Trajectory back = drive.trajectoryBuilder(new Pose2d(intake_nearGrab_x + 2, edgePass_y * side_color, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40, edgePass_y * side_color))
                .build();
        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(40, edgePass_y * side_color, Math.toRadians(180)))
                .lineToLinearHeading(yellowBackDropPosition)
                .build();

        drive.followTrajectorySequence(toMediByLine);
        drive.followTrajectory(moveToCenter);

        upper.autoGrabPrepare(armPos_near1);
        drive.followTrajectory(moveToIntake);
        delay(200);
        upper.autoGrabFinish();

        delay(wait_time);

        drive.followTrajectory(strafe);
        drive.followTrajectory(back);
        upper.setArmPosition(1780);
        drive.followTrajectory(fromProximalToBackdrop);
    }

    protected void delay(int time) {
        long end = System.currentTimeMillis() + time;
        while (opModeIsActive() && end > System.currentTimeMillis()&&update!=null) {
            idle();
            update.run();
        }
    }

    public void distal_edgeBack() {
        Trajectory strafe = drive.trajectoryBuilder(new Pose2d(intake_nearGrab_x, intake_distal_y, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_nearGrab_x + 2, 53 * side_color))
                .build();

        Trajectory back = drive.trajectoryBuilder(new Pose2d(intake_nearGrab_x + 2, 53 * side_color, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40, 53 * side_color))
                .build();

        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(40, 53 * side_color, Math.toRadians(180)))
                .lineToLinearHeading(yellowBackDropPosition)
                .build();
        drive.followTrajectory(strafe);
        drive.followTrajectory(back);
        upper.setArmPosition(1780);
        drive.followTrajectory(fromProximalToBackdrop);
    }
}
