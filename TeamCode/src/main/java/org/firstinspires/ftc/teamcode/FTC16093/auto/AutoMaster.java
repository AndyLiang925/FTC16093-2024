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
    
    public static boolean DEBUG = true;
    public static int startTimeDelay = -1, cycleDelay = -1;
    public static double DESIRED_DISTANCE = 0.4;

    public static final int RED = -1;
    public static final int BLUE = 1;

    protected int startSide;
    protected int side_color;
    protected int drop_side;
    
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
    
    Pose2d startPos;

    public static double startPos_x = 12.125, startPos_y = 59, startPos_heading = 90;
    public static double spikeMark_x, spikeMark_y, spikeMark_heading = 0;

    Pose2d spikeMark = new Pose2d(spikeMark_x, spikeMark_y, Math.toRadians(spikeMark_heading));
    
    public static Pose2d spikeMark_redLeft = new Pose2d(22.5, -45, Math.toRadians(145));
    public static Pose2d spikeMark_redCenter = new Pose2d(24.5, -43, Math.toRadians(110));
    public static Pose2d spikeMark_redRight = new Pose2d(21.7, -54, Math.toRadians(89));//original 95

    public static Pose2d spikeMark_blueLeft = new Pose2d(23.5, 51, Math.toRadians(-90)); //TODO
    public static Pose2d spikeMark_blueCenter = new Pose2d(22, 46, Math.toRadians(-95));
    public static Pose2d spikeMark_blueRight = new Pose2d(22, 45, Math.toRadians(-135));

    public static Pose2d spikeMark_red_distalLeft = new Pose2d(-48.2, -51, Math.toRadians(90));
    public static Pose2d spikeMark_red_distalCenter = new Pose2d(-46, -46, Math.toRadians(85));
    public static Pose2d spikeMark_red_distalRight = new Pose2d(-46, -45, Math.toRadians(45));

    public static Pose2d spikeMark_blue_distalLeft = new Pose2d(-46, 45, Math.toRadians(-40));
    public static Pose2d spikeMark_blue_distalCenter = new Pose2d(-46, 46, Math.toRadians(-85));
    public static Pose2d spikeMark_blue_distalRight = new Pose2d(-48.3, 54, Math.toRadians(-90));
    Pose2d yellowBackDropPosition = new Pose2d(51, 33 * side_color, Math.toRadians(180)); //47.8 blue；51red
    public static int backDrop_heading = 180;
    public static double backDrop_yellowProximal_x = 51; // 47.8 blue; 51 red
    public static double backDrop_yellowDistal_x = 48.8;
    public static double backDrop_white2_x = 49.5;
    public static double backDrop_centerAxis_y = 33.2;
    public static double backDrop_white_y = 26.4; //25.4

    public static double pixel_width = 3.11;
    public static double drop_yellow_index = 0;
    

    private Pose2d getDropPose_proximal(double pos) {
        return new Pose2d(backDrop_yellowProximal_x, side_color * backDrop_centerAxis_y + pos * pixel_width, Math.toRadians(backDrop_heading));
    }
    private Pose2d getDropPose_distal(double pos) {
        return new Pose2d(backDrop_yellowDistal_x, side_color * backDrop_centerAxis_y + pos * pixel_width, Math.toRadians(backDrop_heading));
    } // distal 47.9

    protected Pose2d preDropPose;
    private Pose2d getPreDropPose(double pos){
        return new Pose2d(40, side_color * backDrop_centerAxis_y + pos * pixel_width, Math.toRadians(backDrop_heading));
    }
    protected Pose2d white_preDropPose;
    private Pose2d getWhite_preDropPose(double pos){
        return new Pose2d(40, side_color * backDrop_white_y, Math.toRadians(backDrop_heading));
    }
    private Pose2d whiteDropPos_1;
    private Pose2d getWhiteDropPos_1 ( ){
        return new Pose2d(48, side_color * backDrop_white_y, Math.toRadians(backDrop_heading));
    }

    private Pose2d whiteDropPos_2;
    private Pose2d getWhiteDropPos_2 ( ){
        return new Pose2d(backDrop_white2_x, side_color * backDrop_white_y, Math.toRadians(backDrop_heading));
    }

    public static double intake_far_grab_x = -45, intake_farCenter_y = 8.0;//夹白xy
    public static double gate_y = 6;

    public static double intake_blue_right_oblique_x = -43, intake_blue_right_oblique_y = 32;
    public static double park_x = 43, park_inside = 56, park_outside = 6;
    public static double intake_nearGrab_x = -58.5, intake_distal_y = 10.5, intake_medi_side_x = -53, intake_medi_side_y = 40;  // intake_distal_y Houston = 9.6
    private Pose2d intake_near;
    public static double intake_distal_sideNear_y = 32;
    public static double edgePass_y = 53;
    public static double intake_medi_x = -38;
    public static double intake_medi_y = 6.5;

    public SuperStructure upper;
    public static int armPos_intake1 = 265;
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
//        upper.resetArm();

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

        intake_near = new Pose2d(intake_nearGrab_x, intake_distal_y * side_color, Math.toRadians(180));//在以下改黄片位置
        // index-yellowPosition:
        // 1: center_sideLeft 2:left_sideRight 3: left_sideLeft
        // 0: center_sideRight-1: right_sideRight -2: right_sideRight
        if (startSide == PROXIMAL) {
            if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE) {
                DesiredTagId = 1;
                spikeMark = spikeMark_blueLeft;
                drop_yellow_index = 1;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark = spikeMark_blueCenter;
                drop_yellow_index = -0.5;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark = spikeMark_blueRight;
                drop_yellow_index = -2;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark = spikeMark_redLeft;
                drop_yellow_index = 2.8;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark = spikeMark_redCenter;
                drop_yellow_index = 1.0;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark = spikeMark_redRight;
                drop_yellow_index = - 1.5;
            }
            yellowBackDropPosition = getDropPose_proximal(drop_yellow_index);
            preDropPose = getPreDropPose(drop_yellow_index);
            white_preDropPose = getWhite_preDropPose(drop_yellow_index);
        }

        if (startSide == DISTAL) {
            if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE) {
                DesiredTagId = 1;
                spikeMark = spikeMark_blue_distalLeft;
                drop_yellow_index = 1.5;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark = spikeMark_blue_distalCenter;
                drop_yellow_index = -0.5;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark = spikeMark_blue_distalRight;
                drop_yellow_index = -2.8;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark = spikeMark_red_distalLeft;
                drop_yellow_index = 2.5;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark = spikeMark_red_distalCenter;
                drop_yellow_index = 0.5;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark = spikeMark_red_distalRight;
                drop_yellow_index = -1.5;
            }
            yellowBackDropPosition = getDropPose_distal(drop_yellow_index);
            preDropPose = getPreDropPose(drop_yellow_index);
            white_preDropPose = getPreDropPose(drop_yellow_index);
        }

        whiteDropPos_1 = getWhiteDropPos_1();
        whiteDropPos_2 = getWhiteDropPos_2();
        runtime.reset();
        visionPortal.stopStreaming();
    }

    public void AprilTagDetection() {
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
        upper.setArmPosition(470);
        upper.setSlide(50,0.4);
        drive.setSimpleMoveTolerance(2,0.3);
        drive.setSimpleMovePower(1);
        drive.moveTo(spikeMark, 200);
    }

    public void moveToBackDrop() {
        if (isStopRequested()) return;
        upper.grabPurple(side_color);
        upper.setArmPosition(4100);
        upper.wrist_drop();
        drive.moveTo(preDropPose, 0);
        upper.setArmPosition(4200);
        drive.setSimpleMovePower(0.3);
        drive.setSimpleMoveTolerance(1,0.4);
        drive.moveTo(yellowBackDropPosition, 200);
    }

    public void intakeDistal() {
        if (isStopRequested()) return;
        drive.moveTo(new Pose2d(-57, intake_medi_side_y * side_color, Math.toRadians(180)), 0);
        drive.setSimpleMoveTolerance(2,5);
        drive.moveTo(new Pose2d(-57, intake_distal_y * side_color, Math.toRadians(180)), 0);
        drive.moveTo(new Pose2d(-54, intake_distal_y * side_color, Math.toRadians(180)), 300);
        upper.autoGrabPrepare(armPos_intake1);

        drive.setSimpleMovePower(0.5);
        drive.setSimpleMoveTolerance(2,0.2);
        drive.moveTo(intake_near, 200);
        upper.autoGrabFinish();
        drive.setSimpleMovePower(0.8);
    }
    public void moveToDropWhite(){
        drive.setSimpleMoveTolerance(2.5,0.2);
        drive.moveTo(whiteDropPos_1,200);
    }

    public void distalMoveToBackDrop() {
        drive.setSimpleMoveTolerance(1,0.1);
        drive.moveTo(new Pose2d(25,7*side_color,Math.toRadians(180)),0);
        upper.setArmPosition_slow(3500);
        delay(300);
        upper.setSlide(110,0.8);
        drive.setSimpleMovePower(0.5);
        drive.moveTo(preDropPose,100);
        drive.setSimpleMovePower(0.2);

        upper.setArmPosition(4150);
        drive.setSimpleMoveTolerance(3,2);
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
                .addDisplacementMarker(80, () -> upper.wrist_upwardDrop())
                .addDisplacementMarker(80, () -> upper.setArmPosition(1800))
                .lineToLinearHeading(whiteDropPos_1)
                .build();
        drive.followTrajectorySequence(moveToIntake);
        upper.intake2_lowFar();
        drive.followTrajectorySequence(moveToDrop);
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

        drive.setSimpleMovePower(1);
        if(side == 1){
            drive.moveTo(new Pose2d(park_x, park_inside * side_color, Math.toRadians(-90 * side_color)),0);
            drive.moveTo(new Pose2d(park_x+15,park_inside*side_color,Math.toRadians(-90*side_color)),0);
        }
        if (side == 2) {
            drive.moveTo(new Pose2d(park_x, park_outside * side_color, Math.toRadians(-90 * side_color)),0);
            drive.moveTo(new Pose2d(park_x+15,park_outside*side_color,Math.toRadians(-90*side_color)),0);
        }
    }

    public void setUpAuto() {
        upper.setArmPosition(0);
        upper.resetSlide();
        upper.setSlide(-5, 1);
        upper.grabRight_close();
        upper.grabLeft_close();
        upper.wrist_origin();
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

        drive.followTrajectorySequence(moveToPixel);
        drive.moveTo(new Pose2d(intake_far_grab_x, intake_farCenter_y*side_color, Math.toRadians(180)),100);
        upper.intake2_lowFar();
    }

    public void intakeGate_simpleMove(){
        drive.setSimpleMovePower(0.95);
        drive.setSimpleMoveTolerance(5,3);

        drive.moveTo(new Pose2d(40,gate_y*side_color,Math.toRadians(180)),0);
        drive.moveTo(new Pose2d(-30,intake_farCenter_y*side_color,Math.toRadians(180)),100);
        upper.intakeFar_prep();
        drive.setSimpleMovePower(0.4);
        drive.setSimpleMoveTolerance(4,0.1);
        drive.moveTo(new Pose2d(intake_far_grab_x, intake_farCenter_y*side_color, Math.toRadians(180)),300);
        upper.intakeFar_grab();
    }

    public void moveToDropUpward(){
        drive.setSimpleMovePower(0.95);
        drive.moveTo(new Pose2d(30,intake_farCenter_y*side_color,Math.toRadians(180)),0);
        sleep(100);
        upper.setArmPosition(4300);
        upper.wrist_upwardDrop();
        delay(300);
        drive.moveTo(white_preDropPose,0);
        upper.setArmPosition(4590);
        drive.setSimpleMoveTolerance(2,0.1);
        drive.setSimpleMovePower(0.3);
        drive.moveTo(whiteDropPos_2,100);
    }

    protected void delay(int time) {
        long end = System.currentTimeMillis() + time;
        while (opModeIsActive() && end > System.currentTimeMillis()&&update!=null) {
            idle();
            update.run();
        }
    }
}
