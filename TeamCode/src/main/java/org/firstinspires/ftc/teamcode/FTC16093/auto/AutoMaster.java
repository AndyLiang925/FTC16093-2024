package org.firstinspires.ftc.teamcode.FTC16093.auto;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    public static double startPos_x=12.125,startPos_y=59,startPos_heading=90;
    public static double spikeMark_x,spikeMark_y,spikeMark_heading=0;
    public static double spikeMark_RedLeft_x=12,spikeMark_RedLeft_y= -30;
    public static double spikeMark_RedCenter_x=20.5,spikeMark_RedCenter_y= -20;
    public static double spikeMark_RedRight_x = 36 ,spikeMark_RedRight_y = -30;

    public static double spikeMark_blueLeft_x = 35.0, spikeMark_blueLeft_y = 25; // y initial 30
    public static double spikeMark_blueCenter_x = 26, spikeMark_blueCenter_y = 19;
    public static double spikeMark_blueRight_x = 13,spikeMark_blueRight_y = 25; // y initial 30


    public static double spikeMarkCenter_x,spikeMarkCenter_y;
    public static double spikeMarkCenterProximal_x = 15,spikeMarkCenterProximal_y = 33; // x initial 45
    public static double spikeMarkCenterDistal_x = -59,spikeMarkCenterDistal_y = 30;

    public static double spikeMark_blue_DistalLeft_x = -38, spikeMark_blue_DistalLeft_y = 30.0;
    public static double spikeMark_blue_DistalCenter_x = -45, spikeMark_blue_DistalCenter_y = 21;
    public static double spikeMark_blue_DistalRight_x = -61, spikeMark_blue_DistalRight_y = 30.0;

    public static double spikeMark_Red_DistalLeft_x = -61, spikeMark_Red_DistalLeft_y = -30.0;
    public static double spikeMark_Red_DistalCenter_x = -45, spikeMark_Red_DistalCenter_y = -21;
    public static double spikeMark_Red_DistalRight_x = -38, spikeMark_Red_DistalRight_y = 30.0;

    public static double BackDrop_RedLeft_x = 46.8, BackDrop_RedLeft_y = 22;
    public static double BackDrop_RedCenter_x = 46.8, BackDrop_RedCenter_y = 28;
    public static double BackDrop_RedRight_x = 46.8, BackDrop_RedRight_y = 36.5;
    public static double BackDrop_blueRight_x = 46.8,BackDrop_blueRight_y = 24;
    public static double BackDrop_blueCenter_x = 47.5, BackDrop_blueCenter_y = 29;
    public static double BackDrop_blueLeft_x =46.8,BackDrop_blueLeft_y = 34; //x change to 50, initial 51

    public static double backDrop_blue_DistalRight_x = 47,backDrop_blue_DistalRight_y = 27;
    public static double backDrop_blue_DistalCenter_x = 50.8,backDrop_blue_DistalCenter_y = 36;
    public static double backDrop_blue_distalLeft_x = 47,backDrop_blue_distalLeft_y = 41;

    public static double backDrop_red_distalLeft_x = 47,backDrop_red_distalLeft_y = -27;
    public static double backDrop_red_distalCenter_x = 47,backDrop_red_distalCenter_y = -34;
    public static double backDrop_red_distalRight_x = 47,backDrop_red_distalRight_y = -41;

    public static double detectedBackDrop_x,detectedBackDrop_y, detectedBackDrop_heading=180;


    public static double detectedParking=56;
    public static double intake_x = -45, intake_y = 30.5 ;
    public static double intermediate_y=10,ec_intermediate_y=6;


    public static double ec_backDrop_x, ec_backDrop_y;

    public static boolean closeToIntake = false;

    public static double farPos_x = spikeMark_x, farPos_y = intermediate_y, farPos_heading = 180;

    public static double forwardDistance=3;
    public superstructure upper;
    public static int armPos = 275;

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
        upper.setUp();

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
                ec_backDrop_x = BackDrop_RedRight_x;
                ec_backDrop_y = BackDrop_RedRight_y;
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
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark_x=spikeMark_blue_DistalCenter_x;
                spikeMark_y=spikeMark_blue_DistalCenter_y;
                spikeMark_heading= 0 ;
                detectedBackDrop_x= backDrop_blue_DistalCenter_x;
                detectedBackDrop_y= backDrop_blue_DistalCenter_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark_x = spikeMark_blue_DistalRight_x;
                spikeMark_y = spikeMark_blue_DistalRight_y;
                spikeMark_heading = 0;
                detectedBackDrop_x = backDrop_blue_DistalRight_x;
                detectedBackDrop_y = backDrop_blue_DistalRight_y;

                closeToIntake = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark_x = spikeMark_Red_DistalLeft_x;
                spikeMark_y = spikeMark_Red_DistalLeft_y ;
                spikeMark_heading = 0;
                detectedBackDrop_x = backDrop_red_distalLeft_x;
                detectedBackDrop_y = backDrop_red_distalLeft_y;
                closeToIntake = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark_x = spikeMark_Red_DistalCenter_x;
                spikeMark_y = spikeMark_Red_DistalCenter_y;
                spikeMark_heading = 0;
                detectedBackDrop_x = backDrop_red_distalCenter_x;
                detectedBackDrop_y = backDrop_red_distalCenter_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark_x = spikeMark_Red_DistalRight_x;
                spikeMark_y = spikeMark_blue_DistalRight_y ;
                spikeMark_heading = 0;
                detectedBackDrop_x = backDrop_red_distalRight_x;
                detectedBackDrop_y = backDrop_red_distalRight_y;
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
        //upper.setBox(0.7);
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
        //drive.followTrajectory(moveToVertical);
    }

    public void spikeMarkDump_kickProp(){

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

        drive.followTrajectory(moveToDetectedBackDrop);
    }

    public void DistalBackDropDump(){
        if(isStopRequested()) return;
        Trajectory fromDistalToBack=drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))//remember to change y
                .back(5)
                .build();
        Trajectory fromDistalToIntermediate=drive.trajectoryBuilder(new Pose2d(spikeMark_x-5,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .strafeTo(new Vector2d(spikeMark_x-5,intermediate_y*side_color))
                .build();
        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(spikeMark_x-5,intermediate_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(29.39,intermediate_y*side_color))
                .build();
        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,intermediate_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();

        drive.followTrajectory(fromDistalToBack);
        drive.followTrajectory(fromDistalToIntermediate);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(fromIntermediateToProximal);

        sleep(300);
        drive.followTrajectory(fromProximalToBackdrop);
        sleep(400);
    }

    public void extraCredit(){
        if(isStopRequested()) return;
        Trajectory moveToSide=drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .lineToLinearHeading(new Pose2d(20,54*side_color,Math.toRadians(180)))
                .build();
                /*.splineTo(new Vector2d(20,56.69*side_color),Math.toRadians(163.43))
                .splineTo(new Vector2d(-13.65,60.59*side_color),Math.toRadians(180))
                .splineTo(new Vector2d(-49.61,55*side_color),Math.toRadians(190))
                .splineTo(new Vector2d(intake_x,intake_y*side_color),Math.toRadians(180))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop2_y* side_color))
                .splineTo(new Vector2d(intake_x,detectedBackDrop2_y * side_color),Math.toRadians(180))
                .build();
                 */
        Trajectory moveToDistal = drive.trajectoryBuilder(new Pose2d(20,54*side_color,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36,54*side_color,Math.toRadians(180)))
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

        sleep(200);

        //intake2();
        intake22();//test for new intake with no arm expand

        drive.followTrajectory(fromIntakeToIntermediate);
        drive.followTrajectory(fromIntermediateToProximal);

        sleep(200);
        sleep(300);
        //drive.followTrajectory(fromProximalToSpikeMark);
        drive.followTrajectory(fromSpikeMarkToBackdrop);
    }
    public void extraIntakeSpline(){
        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .splineTo(new Vector2d(8.74, 57.00*side_color), Math.toRadians(180.00))
                .splineTo(new Vector2d(-35.46, 56.26*side_color), Math.toRadians(200.75))
                .splineTo(new Vector2d(intake_x, 37.05), Math.toRadians(180.00))
                .build();


        Trajectory moveToBackDrop = drive.trajectoryBuilder(new Pose2d(intake_x,intake_y,Math.toRadians(0)))
                .splineTo(new Vector2d(-30.40, 12.20), Math.toRadians(-2.59))
                .splineTo(new Vector2d(22.03, 13.94), Math.toRadians(12.80))
                .splineTo(new Vector2d(48.75, 30.55), Math.toRadians(0.00))
                .build();

        drive.followTrajectory(moveToIntake);
        sleep(400);
        intake2();
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(moveToBackDrop);
        drive.turn(Math.toRadians(180));
    }
    public void extraIntakeLinearPath(){
        Trajectory moveToCenter = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
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
        sleep(300);
        intake2();
        sleep(400);
        drive.followTrajectory(moveToBack);
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
        sleep(300);
        intake2();
        sleep(400);
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

    public void parking(){
        if(isStopRequested()) return;
        Trajectory moveToPark=drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .lineTo(new Vector2d((detectedBackDrop_x-3),detectedParking*side_color))
                .build();
        //挪到板子外侧
//        Trajectory moveToBack=drive.trajectoryBuilder(new Pose2d((detectedBackDrop_x-3),detectedParking*side_color,Math.toRadians(spikeMark_heading)))
//                .back(12)
//                .build();
        //////
        //挪到板子内侧
        Trajectory moveToBack=drive.trajectoryBuilder(new Pose2d((detectedBackDrop_x-3),detectedParking*side_color,Math.toRadians(spikeMark_heading)))
                .back(12)
                .build();
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
        upper.setUp();
    }
    public void setKickProp(){
        upper.wrist_to_middle();
    }
    public void putOnSpikeMark(){
        upper.putOnSpikeMark();
    }
    public void putOnBackDrop(){
        upper.putOnBackDrop();
    }
    public void intake1(){

    }
    public void intake2(){
        upper.intake2(armPos);
    }
    public void intake22(){
        upper.autoGrabUpward();
    }

}
