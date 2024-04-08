package org.firstinspires.ftc.teamcode.FTC16093.auto;



import android.text.style.UpdateAppearance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.FTC16093.CenterStageVisionProcessor;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.superstructure;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Config
public class AutoMaster extends LinearOpMode {


    private ElapsedTime runtime;
    public static final int PROXIMAL = 1;
    public static final int DISTAL = -1;
    public static boolean DEBUG = true;
    public static int startTimeDelay = -1, cycleDelay = -1;
    public static double DESIRED_DISTANCE=0.4;

    public static final int RED = -1;
    public static final int BLUE = 1;

    protected int startSide;
    protected int side_color;
    private boolean targetFound;

    private CenterStageVisionProcessor processor;
    private BarkMecanumDrive drive;

    private VisionPortal visionPortal;
    private VisionPortal visionPortalAprilTag;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    private int DesiredTagId=-1;

    private CenterStageVisionProcessor.StartingPosition startingPos;

    final double SPEED_GAIN  =  0.02  ;
    final double STRAFE_GAIN =  0.015 ;
    final double TURN_GAIN   =  0.01  ;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    Pose2d startPos;
    Pose2d farPos;

    public static double startPos_x=12.125, startPos_y=59,startPos_heading=90;
    public static double spikeMark_x,spikeMark_y,spikeMark_heading=0;
    public static double spikeMark_RedLeft_x=12,spikeMark_RedLeft_y= -30;
    public static double spikeMark_RedCenter_x=20.5,spikeMark_RedCenter_y= -20;
    public static double spikeMark_RedRight_x = 36 ,spikeMark_RedRight_y = -30;

    public static double spikeMark_blueLeft_x = 35.0, spikeMark_blueLeft_y = 26; // y initial 30
    public static double spikeMark_blueCenter_x = 26, spikeMark_blueCenter_y = 21;
    public static double spikeMark_blueRight_x = 13,spikeMark_blueRight_y = 30; // y initial 30


    public static double spikeMarkCenter_x,spikeMarkCenter_y;
    public static double spikeMarkCenterProximal_x = 20,spikeMarkCenterProximal_y = 33; // x initial 45
    public static double spikeMarkCenterDistal_x = -59,spikeMarkCenterDistal_y = 30;

    public static double spikeMark_blue_DistalLeft_x = -36, spikeMark_blue_DistalLeft_y = 27;
    public static double spikeMark_blue_DistalCenter_x = -43, spikeMark_blue_DistalCenter_y = 15;
    public static double spikeMark_blue_DistalRight_x = -37, spikeMark_blue_DistalRight_y = 30;

    public static double spikeMark_Red_DistalLeft_x = -35, spikeMark_Red_DistalLeft_y = -20;
    public static double spikeMark_Red_DistalCenter_x = -43, spikeMark_Red_DistalCenter_y = -15;
    public static double spikeMark_Red_DistalRight_x = -35, spikeMark_Red_DistalRight_y = -20;

    public static double BackDrop_RedLeft_x = 46.8, BackDrop_RedLeft_y = -22;
    public static double BackDrop_RedCenter_x = 46.8, BackDrop_RedCenter_y = -28;
    public static double BackDrop_RedRight_x = 46.8, BackDrop_RedRight_y = -36.5;
    public static double BackDrop_blueRight_x = 47.3,BackDrop_blueRight_y = 24;
    public static double BackDrop_blueCenter_x = 47.5, BackDrop_blueCenter_y = 29;
    public static double BackDrop_blueLeft_x =47.3,BackDrop_blueLeft_y = 34; //x change to 50, initial 51

    public static double backDrop_blue_distalRight_x = 48.5,backDrop_blue_distalRight_y = 23;
    public static double backDrop_blue_distalCenter_x = 48.5,backDrop_blue_distalCenter_y = 28; //
    public static double backDrop_blue_distalLeft_x = 48,backDrop_blue_distalLeft_y = 34;

    public static double backDrop_red_distalLeft_x = 47,backDrop_red_distalLeft_y = -24;
    public static double backDrop_red_distalCenter_x = 47,backDrop_red_distalCenter_y = -29;
    public static double backDrop_red_distalRight_x = 47,backDrop_red_distalRight_y = -41;

    public static double detectedBackDrop_x,detectedBackDrop_y, detectedBackDrop_heading=180;

    public static double ec_backDrop_blue_distalLeft_y = 24;
    public static double intake_blue_near_x = -57, intake_blue_left_near_y = 9;
    public static double intake_blue_center_near_y = 18;

    public static double intake_blue_right_oblique_x = - 48, intake_blue_right_oblique_y = 45,intake_oblique_heading_grab1= -135;
    public static double intake_red_near_x = - 57, intake_red_left_near_y = 18;

    public static double intake_red_center_near_y = - 57, intake_red_right_near_y = -6;

    public static double intake_blue_far_x = -46;


    public static double park_x = 43, park_inside=56 , park_outside = 6;

    public static double intake_x = -45, intake_y = 30.5 ;
    public static double intake_oblique_blue_x = -50, intake_oblique_blue_y = 11;
    public static double intake_near_x = -58.5, intake_near_y = 9;
    public static double intake_medi_x = -35;
    public static double intake_center_far_x= -43, intake_center_far_y= 34;

    public static double intermediate_y=5, ec_intermediate_y=6;

    public static double edge_medi_y = 57;
    public static double ec_backDrop_x, ec_backDrop_y;

    public static boolean closeToIntake = false;

    public static double forwardDistance=3;
    public superstructure upper;
    public static int armPos = 280,armPos_near1 = 140, armPos_near_low = 50, armPos_far_low = 100;//175
    public static int wait_time = 800,sleep_2=200,sleep_3=1000;

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
        if(startSide==PROXIMAL) {
            startPos = new Pose2d(startPos_x * startSide, startPos_y * side_color, Math.toRadians(startPos_heading * side_color));
        }else{
            startPos = new Pose2d((startPos_x+24)*startSide,startPos_y * side_color,Math.toRadians(startPos_heading*side_color));
        }
        drive = new BarkMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(startPos);
        drive.update();
        drive.getLocalizer().setPoseEstimate(startPos);
        drive.update();
        telemetry.addLine("init: superstructure");
        upper = new superstructure(this,drive::update);
        setUpAuto();

        telemetry.addLine("init: trajectory");


        while (!opModeIsActive()) {
            startingPos = processor.getStartingPosition();
            telemetry.addData("Vision", startingPos);
            telemetry.update();
            long time = System.currentTimeMillis();
            telemetry.update();
            sleep(15);
            while (System.currentTimeMillis() - time < 100 && opModeInInit()) idle();
            if (isStopRequested()) throw new InterruptedException();
        }
        waitForStart();

        if(startSide==PROXIMAL) {
            spikeMarkCenter_x=spikeMarkCenterProximal_x;
            spikeMarkCenter_y=spikeMarkCenterProximal_y;

            if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE) {
                DesiredTagId = 1;
                spikeMark_x=spikeMark_blueLeft_x ;
                spikeMark_y=spikeMark_blueLeft_y;

                spikeMark_heading=180;
                detectedBackDrop_x= BackDrop_blueLeft_x;
                detectedBackDrop_y= BackDrop_blueLeft_y;

                ec_backDrop_x = BackDrop_blueRight_x;
                ec_backDrop_y = BackDrop_blueRight_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark_x=spikeMark_blueCenter_x;
                spikeMark_y=spikeMark_blueCenter_y;
                spikeMark_heading=180;
                detectedBackDrop_x= BackDrop_blueCenter_x;
                detectedBackDrop_y= BackDrop_blueCenter_y;
                ec_backDrop_x = BackDrop_blueRight_x;
                ec_backDrop_y = BackDrop_blueRight_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark_x=spikeMark_blueRight_x;
                spikeMark_y=spikeMark_blueRight_y;
                spikeMark_heading=180;
                detectedBackDrop_x= BackDrop_blueRight_x;
                detectedBackDrop_y= BackDrop_blueRight_y;
                ec_backDrop_x = BackDrop_blueLeft_x;
                ec_backDrop_y = BackDrop_blueLeft_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark_x = spikeMark_RedLeft_x  ;
                spikeMark_y = spikeMark_RedLeft_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = BackDrop_RedLeft_x;
                detectedBackDrop_y = BackDrop_RedLeft_y ;

                ec_backDrop_x = BackDrop_RedRight_x;
                ec_backDrop_y = BackDrop_RedRight_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark_x = spikeMark_RedCenter_x ;
                spikeMark_y = spikeMark_RedCenter_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = BackDrop_RedCenter_x;
                detectedBackDrop_y = BackDrop_RedCenter_y ;

                ec_backDrop_x = BackDrop_RedLeft_x;
                ec_backDrop_y = BackDrop_RedLeft_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark_x = spikeMark_RedRight_x ;
                spikeMark_y = spikeMark_RedRight_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = BackDrop_RedRight_x;
                detectedBackDrop_y = BackDrop_RedRight_y ;

                ec_backDrop_x = BackDrop_RedLeft_x;
                ec_backDrop_y = BackDrop_RedLeft_y;
            }
        }
        if(startSide==DISTAL){
            spikeMarkCenter_x = spikeMarkCenterDistal_x;
            spikeMarkCenter_y = spikeMarkCenterDistal_y;
            if(startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE){
                DesiredTagId=1;
                spikeMark_x=spikeMark_blue_DistalLeft_x;
                spikeMark_y=spikeMark_blue_DistalLeft_y;
                spikeMark_heading= 0;
                detectedBackDrop_x= backDrop_blue_distalLeft_x;
                detectedBackDrop_y= backDrop_blue_distalLeft_y;

                intake_near_y = intake_blue_left_near_y;
                ec_backDrop_x = backDrop_blue_distalRight_x;
                ec_backDrop_y = ec_backDrop_blue_distalLeft_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark_x=spikeMark_blue_DistalCenter_x;
                spikeMark_y=spikeMark_blue_DistalCenter_y;
                spikeMark_heading= 0 ;
                detectedBackDrop_x= backDrop_blue_distalCenter_x;
                detectedBackDrop_y= backDrop_blue_distalCenter_y;

                intake_near_y = intake_blue_left_near_y;
                ec_backDrop_x = backDrop_blue_distalRight_x;
                ec_backDrop_y = backDrop_blue_distalRight_y;

            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark_x = spikeMark_blue_DistalRight_x;
                spikeMark_y = spikeMark_blue_DistalRight_y;
                spikeMark_heading = 180;
                detectedBackDrop_x = backDrop_blue_distalRight_x;
                detectedBackDrop_y = backDrop_blue_distalRight_y;

                intake_near_y = intake_blue_left_near_y;
                ec_backDrop_x = backDrop_blue_distalLeft_x;
                ec_backDrop_y = backDrop_blue_distalLeft_y;
                closeToIntake = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark_x = spikeMark_Red_DistalLeft_x;
                spikeMark_y = spikeMark_Red_DistalLeft_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = backDrop_red_distalLeft_x;
                detectedBackDrop_y = backDrop_red_distalLeft_y;

                intake_near_y = intake_red_right_near_y;
                ec_backDrop_x = backDrop_red_distalRight_x;
                ec_backDrop_y = backDrop_red_distalRight_y;
                closeToIntake = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark_x = spikeMark_Red_DistalCenter_x;
                spikeMark_y = spikeMark_Red_DistalCenter_y;
                spikeMark_heading = 0;
                detectedBackDrop_x = backDrop_red_distalCenter_x;
                detectedBackDrop_y = backDrop_red_distalCenter_y;

                intake_near_y = intake_red_right_near_y;
                ec_backDrop_x = backDrop_red_distalLeft_x;
                ec_backDrop_y = backDrop_red_distalLeft_y;

            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark_x = spikeMark_Red_DistalRight_x;
                spikeMark_y = spikeMark_Red_DistalRight_y ;
                spikeMark_heading = 0;
                detectedBackDrop_x = backDrop_red_distalRight_x;
                detectedBackDrop_y = backDrop_red_distalRight_y;

                intake_near_y = intake_red_right_near_y;
                ec_backDrop_x = backDrop_red_distalLeft_x;
                ec_backDrop_y = backDrop_red_distalLeft_y;
            }
        }
        runtime.reset();
        visionPortal.stopStreaming();
    }

    public void ApriltagDetection(){
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortalAprilTag = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();
        targetFound = false;
        desiredTag  = null;

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
    public void moveToCenter(){
        if (isStopRequested()) return;
        Trajectory moveToCenter;
        moveToCenter = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(spikeMarkCenter_x,spikeMarkCenter_y*side_color,Math.toRadians(spikeMark_heading)))
                .build();
        drive.followTrajectory(moveToCenter);

    }
    public void spikeMarkDump(){
        if (isStopRequested()) return;
        Trajectory moveToSpikeMark;
        moveToSpikeMark = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(spikeMark_x, spikeMark_y,Math.toRadians(spikeMark_heading)))
                .build();
        drive.followTrajectory(moveToSpikeMark);
    }

    public void resetUpper(){
        drive.resetEncoder();
    }


    public void barkKickProp(){
        if (isStopRequested()) return;
        drive.turn(Math.toRadians(30));
        sleep(300);
        drive.turn(Math.toRadians(-30));
        //drive.followTrajectory(moveTo0ertical);
    }

    public void backDropDump(){
        if(isStopRequested()) return;
        Trajectory moveToDetectedBackDrop=drive.trajectoryBuilder(new Pose2d(spikeMark_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();

        upper.setArmPosition(1800);
        drive.followTrajectory(moveToDetectedBackDrop);
    }

    public void DistalBackDropDump(){
        if(isStopRequested()) return;
        Trajectory toMediByLine = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .lineToLinearHeading(new Pose2d(intake_medi_x,intake_near_y*side_color,Math.toRadians(180)))
                .build();

        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(intake_medi_x,intake_near_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_near_x,intake_near_y*side_color))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_near_x,intake_near_y*side_color,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(29.39,intermediate_y*side_color))
                .build();
        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,intermediate_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();
        drive.followTrajectory(toMediByLine);

        upper.autoGrabPrepare(armPos_near1);
        drive.followTrajectory(moveToIntake);
        upper.autoGrabFinish();

        drive.followTrajectory(fromIntermediateToProximal);
        upper.setArmPosition(1780);
        drive.followTrajectory(fromProximalToBackdrop);
    }

    public void DistalBackDropDump_farGrab(){
        if(isStopRequested()) return;
        Trajectory toIntakeByLine = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .lineToLinearHeading(new Pose2d(intake_center_far_x,intake_center_far_y*side_color,Math.toRadians(180)))
                .build();
        Trajectory toMedi = drive.trajectoryBuilder(new Pose2d(intake_center_far_x,intake_center_far_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_x,intake_near_y*side_color))
                .build();
        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_x,intake_near_y*side_color,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(29.39,intermediate_y*side_color))
                .build();
        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,intermediate_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();

        drive.followTrajectory(toIntakeByLine);
        intake1_byGrab1();
        drive.followTrajectory(toMedi);
        drive.followTrajectory(fromIntermediateToProximal);
        upper.setArmPosition(1880); //1950
        drive.followTrajectory(fromProximalToBackdrop);
    }
    public void extraCredit(){
        if(isStopRequested()) return;
        Trajectory moveToSide=drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .lineToLinearHeading(new Pose2d(20,54*side_color,Math.toRadians(180)))
                .build();

        Trajectory moveToDistal = drive.trajectoryBuilder(new Pose2d(20,54*side_color,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36,54*side_color,Math.toRadians(-179)))
                .build();

        Trajectory moveToPixel=drive.trajectoryBuilder(new Pose2d(-36,54*side_color,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(intake_x,intake_y*side_color))
                .build();

        Trajectory moveToIntake=drive.trajectoryBuilder(new Pose2d(-44,intake_y*side_color,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(intake_x,intake_y*side_color))
                .build();

        Trajectory fromIntakeToIntermediate=drive.trajectoryBuilder(new Pose2d(intake_x,intake_y*side_color,Math.toRadians(spikeMark_heading)))
                .strafeTo(new Vector2d(intake_x,ec_intermediate_y*side_color))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_x,ec_intermediate_y*side_color,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(29.39,ec_intermediate_y*side_color))
                .build();
        Trajectory fromProximalToSpikeMark = drive.trajectoryBuilder(new Pose2d(29.39,ec_intermediate_y*side_color,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(29.39,detectedBackDrop_y*side_color))
                .build();

        Trajectory fromSpikeMarkToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,ec_intermediate_y*side_color,Math.toRadians(spikeMark_heading)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y*side_color))
                .build();

        drive.followTrajectory(moveToSide);
        drive.followTrajectory(moveToDistal);
        drive.followTrajectory(moveToPixel);

        //sleep(200);

        intake2();//test for new intake with no arm expand

        drive.followTrajectory(fromIntakeToIntermediate);
        drive.followTrajectory(fromIntermediateToProximal);
        //drive.followTrajectory(fromProximalToSpikeMark);
        drive.followTrajectory(fromSpikeMarkToBackdrop);
    }
    public void ec_lowFar_edgeSpline_blue(){
        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .splineTo(new Vector2d(8.74, 55*side_color), Math.toRadians(180.00))
                .splineTo(new Vector2d(-35.46, 55*side_color), Math.toRadians(180))
                .splineTo(new Vector2d(intake_blue_right_oblique_x, intake_blue_right_oblique_y), Math.toRadians(intake_oblique_heading_grab1))
                .build();

        Trajectory moveToBack= drive.trajectoryBuilder(new Pose2d(intake_blue_right_oblique_x,intake_blue_right_oblique_y,Math.toRadians(intake_oblique_heading_grab1)))
                .lineToLinearHeading(new Pose2d(-12,55*side_color,Math.toRadians(180)))
                .build();

        Trajectory back= drive.trajectoryBuilder(new Pose2d(-12,55*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40,55*side_color))
                .build();

        Trajectory drop= drive.trajectoryBuilder(new Pose2d(40,55*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();

        drive.followTrajectory(moveToIntake);
        sleep(400);
        intake2_lowfar_grab1();

        drive.followTrajectory(moveToBack);
        drive.followTrajectory(back);
        drive.followTrajectory(drop);
    }
    public void extraIntakeLinearPath(){
        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(intake_x,intake_y))
                .build();
        Trajectory moveToDrop = drive.trajectoryBuilder(new Pose2d(intake_x, intake_y,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();
        //drive.followTrajectory(moveToCenter);
        drive.followTrajectory(moveToIntake);
        //sleep(300);
        //drive.correct_heading(0);
        intake2();
        drive.followTrajectory(moveToDrop);
        upper.setArmPosition(1890);
        sleep(1000);
    }
    public void extraIntakeLinearPath2plus4(){
        Trajectory moveToCenter = drive.trajectoryBuilder(new Pose2d(ec_backDrop_x,ec_backDrop_y,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(spikeMarkCenter_x, spikeMarkCenter_y*side_color))
                .build();
        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(spikeMarkCenter_x, spikeMarkCenter_y*side_color,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(intake_x,intake_y))
                .build();

        Trajectory moveToBack = drive.trajectoryBuilder(new Pose2d(intake_x, intake_y,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(spikeMarkCenter_x,spikeMarkCenter_y*side_color))
                .build();
        Trajectory moveToDrop = drive.trajectoryBuilder(new Pose2d(spikeMarkCenter_x,spikeMarkCenter_y,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();
        drive.followTrajectory(moveToCenter);

        drive.followTrajectory(moveToIntake);
        //sleep(300);
        //drive.correct_heading(0);
        intake2();
        drive.followTrajectory(moveToBack);
        upper.setArmPosition(1890);
        drive.followTrajectory(moveToDrop);
    }

    public void extraIntakeLinearBySpline(){
        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .splineTo(new Vector2d(spikeMarkCenter_x, spikeMarkCenter_y*side_color), Math.toRadians(183.52))
                .splineTo(new Vector2d(intake_x, intake_y*side_color), Math.toRadians(180.00))
                .build();

        Trajectory moveToBack = drive.trajectoryBuilder(new Pose2d(intake_x,intake_y*side_color,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(spikeMarkCenter_x,spikeMarkCenter_y))
                .build();
        Trajectory moveToDrop = drive.trajectoryBuilder(new Pose2d(spikeMarkCenter_x,spikeMarkCenter_y,Math.toRadians(spikeMark_heading)))
                .lineToConstantHeading(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();
        drive.followTrajectory(moveToIntake);
        //sleep(300);
        intake2();
        //sleep(400);
        drive.followTrajectory(moveToBack);
        drive.followTrajectory(moveToDrop);
    }
    public void forward(){
        Trajectory moveForward = drive.trajectoryBuilder(new Pose2d(intake_x,intake_y*side_color,Math.toRadians(180)))
                .forward(forwardDistance)
                .build();
        drive.followTrajectory(moveForward);
    }
    public void back(){
        Trajectory moveBack = drive.trajectoryBuilder(new Pose2d(intake_x + forwardDistance,intake_y*side_color,Math.toRadians(180)))
                .back(forwardDistance)
                .build();
        drive.followTrajectory(moveBack);
    }

    public void parking(int side){//side==1: inside         side==2: outside
        if(isStopRequested()) return;
        Trajectory moveToPark = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,ec_backDrop_y,Math.toRadians(180)))
                .lineTo(new Vector2d(park_x,park_inside*side_color))
                .build();
        Trajectory moveToBack=drive.trajectoryBuilder(new Pose2d(park_x,park_inside*side_color,Math.toRadians(180)))
                .back(12)
                .build();
        if (side==2){
            moveToPark = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,ec_backDrop_y,Math.toRadians(180)))
                    .lineTo(new Vector2d(park_x,park_outside*side_color))
                    .build();
            moveToBack=drive.trajectoryBuilder(new Pose2d(park_x,park_outside*side_color,Math.toRadians(180)))
                    .back(12)
                    .build();
        }

        drive.followTrajectory(moveToPark);
        drive.followTrajectory(moveToBack);

    }

    public void headReset(){
        drive.turn(Math.toRadians(-90));
    }

    public void driveToAprilTag(){
        boolean tracksuccess = false;
        while(opModeIsActive()&&targetFound&&!tracksuccess){
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            if(rangeError>0&&headingError>0&&yawError>0){
                tracksuccess=false;
            }else{
                tracksuccess=true;
            }
        }
    }

    public void setUpAuto(){
        upper.setArmPosition(0);
        upper.setArmLength(0);
        upper.grab1_close();//gb1.setPosition(0.22);
        upper.grab2_close();//gb2.setPosition(0.45);
        upper.wrist_to_middle();
    }
    public void setKickProp(){
        upper.wrist_to_middle();
    }
    public void putOnSpikeMark(){
        upper.wristDown();
        upper.setArmPosition(30);
        sleep(300);
        upper.grab1_open();
        upper.wrist_to_middle();
        //upper.setArmPosition(1890);
        sleep(300);
    }
    public void putOnBackDrop_grab2(){
        upper.grab2_open();
        sleep(200);
        upper.grab2_close();
    }
    public void raiseArm(){
        upper.setArmPosition(2000);
        sleep(1500);
    }
    public void backDrop_move(){
        Trajectory moveToAnotherDrop = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(detectedBackDrop_x,ec_backDrop_y))
                .build();
        drive.followTrajectory(moveToAnotherDrop);
    }

    public void putOnBackDrop_grab1(){
        upper.grab1_open(); //white
        sleep(200);
    }

    public void intake2_grab1_near_prepare(){
        upper.setArmPosition(armPos_near_low);
        sleep(200);
        upper.grab1_open();
        upper.wristDown();
    }


    public void intake_throw(){
        upper.setArmLength(300);
        upper.setArmPosition(350);
        sleep(300);
        upper.wrist_to_down();
        upper.setArmLength(0);
        sleep(300);
        upper.wrist_to_middle();
        sleep(300);
        upper.setArmPosition(10);
    }

    public void intake2(){
        upper.wrist_grab2_distalAuto(armPos);
        sleep(500);
        upper.grab2_close();
        sleep(200);
        intake_throw();
        sleep(200);
        upper.wrist_to_middle();
        sleep(200);
        upper.setArmLength(0);
        sleep(200);
        upper.setArmPosition(0);
    }
    public void intake2_lowfar_grab1(){
        upper.wrist_grab1_distalAuto(armPos_far_low);
        sleep(900);
        upper.grab1_close();
        sleep(200);
        intake_throw();
        sleep(200);
        upper.setArmLength(0);
        sleep(200);
        upper.setArmPosition(0);
    }
    public void intake2_lowFar_grab2(){
        upper.wrist_grab2_distalAuto(armPos_far_low);
        sleep(900);
        upper.grab2_close();
        sleep(200);
        intake_throw();
        sleep(200);
        upper.setArmLength(0);
        sleep(200);
        upper.setArmPosition(0);
    }


    public void intake2_byGrab1(){
        upper.wrist_grab1_distalAuto(armPos);
        sleep(500);
        upper.grab1_close();
        sleep(200);
        intake_throw();
        sleep(200);
        upper.wrist_to_middle();
        sleep(200);
        upper.setArmLength(0);
        sleep(200);
        upper.setArmPosition(0);
    }
    public void intake2_lowNear_byGrab1(){
        upper.wrist_grab1_distalAuto(armPos_near_low);
        sleep(500);
        upper.grab1_close();
        sleep(200);
        intake_throw();
        sleep(200);
        upper.wrist_to_middle();
        sleep(200);
        upper.setArmLength(0);
        sleep(200);
        upper.setArmPosition(0);
    }
    public void dropGround(){
        upper.wrist_grab1_distalAuto(armPos);
        sleep(500);
        upper.grab1_open();
        sleep(200);
        intake_throw();
        sleep(200);
        upper.wrist_to_middle();
        sleep(200);
        upper.setArmLength(0);
        sleep(200);
        upper.setArmPosition(0);
    }
    public void dropGround_low(){
        upper.wristDown();
        sleep(100);
        upper.grab1_open();
        sleep(200);
        upper.wrist_to_middle();
    }
    public void intake1_byGrab1(){
        upper.wrist_grab1_distalAuto(armPos);
        sleep(500);
        upper.grab1_close();
        sleep(200);
        upper.wrist_to_middle();
        sleep(200);
        upper.setArmLength(0);
        sleep(200);
        upper.setArmPosition(0);
    }

    public void ecByCenter(){
        Trajectory intake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(10,intake_near_y*side_color),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_x, intake_near_y*side_color))
                .build();

        Trajectory moveForward = drive.trajectoryBuilder(new Pose2d(intake_x,intake_near_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_near_x-2,intake_near_y*side_color))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_near_x-2,intake_near_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(35,intermediate_y*side_color))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(35, intermediate_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();

        drive.followTrajectory(intake);
        intake2_grab1_near_prepare();
        drive.followTrajectory(moveForward);

        upper.grab1_close();
        sleep(150);
        upper.wrist_to_middle();
        upper.setArmPosition(0);
        drive.followTrajectory(fromIntermediateToProximal);
        drive.followTrajectory(drop);
    }

    public void ecByCenter_far(){
        Trajectory intake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(10,intake_near_y*side_color),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_x, intake_near_y*side_color))
                .build();
        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_x,intake_near_y*side_color,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(10,intermediate_y*side_color))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(30, intermediate_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();

        drive.followTrajectory(intake);
        intake2_byGrab1();
        drive.followTrajectory(fromIntermediateToProximal);
        drive.followTrajectory(drop);
    }

    public void ecByCenter_farCenter(){
        Trajectory moveToPixel = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(10,intermediate_y*side_color),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_medi_x, intermediate_y*side_color))
                .build();

        Trajectory intake = drive.trajectoryBuilder(new Pose2d(intake_medi_x,intermediate_y*side_color, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(intake_oblique_blue_x,intake_oblique_blue_y*side_color,Math.toRadians(165)))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_oblique_blue_x,intake_oblique_blue_y*side_color,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(30,intermediate_y*side_color))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(30, intermediate_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();

        drive.followTrajectory(moveToPixel);
        drive.followTrajectory(intake);
        intake2_lowFar_grab2();
        drive.followTrajectory(fromIntermediateToProximal);
        drive.followTrajectory(drop);
    }

    public void ec_far_putOnGround(){
        Trajectory intake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(10,intake_near_y*side_color),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_x, intake_near_y*side_color))
                .build();
        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_x,intake_near_y*side_color,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(45,intermediate_y*side_color,Math.toRadians(0)))
                .build();

        drive.followTrajectory(intake);
        intake2_lowfar_grab1();
        drive.followTrajectory(fromIntermediateToProximal);
        dropGround_low();
    }

}
