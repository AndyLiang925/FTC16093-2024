package org.firstinspires.ftc.teamcode.FTC16093;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.FTC16093.uppersystem.superstructure;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Config
public class AutoMaster extends LinearOpMode {


    private ElapsedTime runtime; // 1111////right
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

    final double MAX_AUTO_SPEED = 0.5; //0.5
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    Pose2d startPos;
    Pose2d farPos;

    public static double startPos_x=12.125,startPos_y=59,startPos_heading=90;
    public static double spikeMark_x,spikeMark_y,spikeMark_heading=0;
    public static double spikeMark1_x=12,spikeMark1_y=30;
    public static double spikeMark2_x=20.5,spikeMark2_y=20; // spikeMark2 initial 23, change to 21.5
    public static double spikeMark3_x=36,spikeMark3_y=30; // spikeMark3 initial 33.5 change to 31.5

    public static double spikeMark1_x_Blue = 35.0;
    public static double spikeMark2_x_Blue = 26;
    public static double spikeMark3_x_Blue = 13; //10?

    public static double spikeMarkCenter_x,spikeMarkCenter_y;
    public static double spikeMarkCenterProximal_x = 45,spikeMarkCenterProximal_y = 30;
    public static double spikeMarkCenterDistal_x = -59,spikeMarkCenterDistal_y = 30;

    public static double spikeMark1_x_Blue_Distal = -38, spikeMark1_y_Blue_Distal = 30.0;  //x:11+24
    public static double spikeMark2_x_Blue_Distal = -45, spikeMark2_y_Blue_Distal = 21;  // x:21+24
    public static double spikeMark3_x_Blue_Distal = -61, spikeMark3_y_Blue_Distal = 30.0; // x:30+24

    public static double detectedBackDrop1_x = 46.8,detectedBackDrop1_y = 22;
    public static double detectedBackDrop2_x = 46.8,detectedBackDrop2_y = 28;
    public static double detectedBackDrop3_x = 46.8,detectedBackDrop3_y = 36.5;
    public static double detectedBackDrop1_x_Blue = 46.8,detectedBackDrop1_y_Blue = 24;
    public static double detectedBackDrop2_x_Blue = 46.8,detectedBackDrop2_y_Blue = 29;
    public static double detectedBackDrop3_x_Blue =46.8,detectedBackDrop3_y_Blue = 34; //x change to 50, initial 51

    public static double BackDrop1_distalBlue_x = 47,BackDrop1_distalBlue_y = 27;
    public static double BackDrop2_distalBlue_x = 50.8,BackDrop2_distalBlue_y = 36;
    public static double BackDrop3_distalBlue_x = 47,BackDrop3_distalBlue_y = 41;

    public static double BackDrop1_distalRed_x = 47,BackDrop1_distalRed_y = 27; // initial y= 44, change  with 3
    public static double BackDrop2_distalRed_x = 47,BackDrop2_distalRed_y = 34;
    public static double BackDrop3_distalRed_x = 47,BackDrop3_distalRed_y = 41;


    // public static double BackDrop_distalBlue_x = 51, BackDrop_distalBlue_y = 27;
    // public static double BackDrop_distalRed_x = 51, BackDrop_distalRed_y = 27;
    public static double detectedBackDrop_x,detectedBackDrop_y, detectedBackDrop_heading=180;


    public static double detectedParking=56; // add by me 外侧值:56
    public static double intake_x = -48, intake_y = 36.90 ;

    public static double duration_timeM = 1000, intake_power= -0.17;
    public static double intake_duration1_timeM = 500, intake_duration2_timeM = 150, intake_realPower = 1;

    public static double intermediate_y=10,ec_intermediate_y=6;
    public static double ec_backDropRed_x = 45, ec_backDropRed_y = 27;
    public static double ec_backDropBlue_x = 45, ec_backDropBlue_y = 27;

    public static double ec_final_backDrop_x, ec_final_backDrop_y;
    public static boolean closeToIntake = false;

    public static double farPos_x = spikeMark_x, farPos_y = intermediate_y, farPos_heading = 180;

    public static double forwardDistance=3;
    private superstructure upper;
    public static int armPos=250;

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
//        spikeMarkCenter_x=spikeMarkCenterProximal_x;
//        spikeMarkCenter_y=spikeMarkCenterProximal_y;
//        ec_final_backDrop_x = ec_backDropRed_x;
//        ec_final_backDrop_y = ec_backDropRed_y;
//        DesiredTagId = 4;
//        spikeMark_x = spikeMark1_x * PROXIMAL;
//        spikeMark_y = spikeMark1_y * RED;
//        spikeMark_heading = 183;
//        detectedBackDrop_x = detectedBackDrop1_x;
//        detectedBackDrop_y = detectedBackDrop1_y * RED;

        if(startSide==PROXIMAL) {
            spikeMarkCenter_x=spikeMarkCenterProximal_x;
            spikeMarkCenter_y=spikeMarkCenterProximal_y;
            if(side_color == BLUE){
                ec_final_backDrop_x= ec_backDropBlue_x;
                ec_final_backDrop_y = ec_backDropBlue_y;
            }else{
                ec_final_backDrop_x = ec_backDropRed_x;
                ec_final_backDrop_y = ec_backDropRed_y;
            }

            if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE) {
                DesiredTagId = 1;
                spikeMark_x=spikeMark1_x_Blue * PROXIMAL;
                spikeMark_y=spikeMark3_y * BLUE;
                spikeMark_heading=180;
                detectedBackDrop_x=detectedBackDrop3_x_Blue; // x will not change so delete Proximal
                detectedBackDrop_y=detectedBackDrop3_y_Blue;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark_x=spikeMark2_x_Blue*PROXIMAL;
                spikeMark_y=spikeMark2_y*BLUE;
                spikeMark_heading=180;
                detectedBackDrop_x=detectedBackDrop2_x_Blue; //
                detectedBackDrop_y=detectedBackDrop2_y_Blue;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark_x=spikeMark3_x_Blue*PROXIMAL;
                spikeMark_y=spikeMark1_y*BLUE;
                spikeMark_heading=180;
                detectedBackDrop_x=detectedBackDrop1_x_Blue;
                detectedBackDrop_y=detectedBackDrop1_y_Blue;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark_x = spikeMark1_x * PROXIMAL;
                spikeMark_y = spikeMark1_y * RED;
                spikeMark_heading = 180;
                detectedBackDrop_x = detectedBackDrop1_x;
                detectedBackDrop_y = detectedBackDrop1_y * RED;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark_x = spikeMark2_x * PROXIMAL;
                spikeMark_y = spikeMark2_y * RED; // initial spikeMark1_y change to Mark2_y
                spikeMark_heading = 180;
                detectedBackDrop_x = detectedBackDrop2_x;
                detectedBackDrop_y = detectedBackDrop2_y * RED;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark_x = spikeMark3_x * PROXIMAL;
                spikeMark_y = spikeMark3_y * RED;
                spikeMark_heading = 180;
                detectedBackDrop_x = detectedBackDrop3_x;
                detectedBackDrop_y = detectedBackDrop3_y * RED;
            }
        }
        if(startSide==DISTAL){
            spikeMarkCenter_x = spikeMarkCenterDistal_x;
            spikeMarkCenter_y = spikeMarkCenterDistal_y;
            if(startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE){
                DesiredTagId=1; // symmetrical to the proximal 1
                spikeMark_x=spikeMark1_x_Blue_Distal;
                spikeMark_y=spikeMark1_y_Blue_Distal;
                spikeMark_heading=-3; // initial 177
                detectedBackDrop_x=BackDrop3_distalBlue_x;
                detectedBackDrop_y=BackDrop3_distalBlue_y;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark_x=spikeMark2_x_Blue_Distal;
                spikeMark_y=spikeMark2_y_Blue_Distal;
                spikeMark_heading=-5; // initial 175
                detectedBackDrop_x=BackDrop2_distalBlue_x;
                detectedBackDrop_y=BackDrop2_distalBlue_y;//31
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark_x = spikeMark3_x_Blue_Distal;
                spikeMark_y = spikeMark3_y_Blue_Distal; // doesn't need spikeMark3_y, since 1 and 3 y are completely the same
                spikeMark_heading = -3; // initial 177
                detectedBackDrop_x = BackDrop1_distalBlue_x;
                detectedBackDrop_y = BackDrop1_distalBlue_y;//25.5
                closeToIntake = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark_x = spikeMark3_x_Blue_Distal;
                spikeMark_y = spikeMark3_y_Blue_Distal * RED;
                spikeMark_heading = 0; // initial 183
                detectedBackDrop_x = BackDrop1_distalRed_x; // change to distal
                detectedBackDrop_y = BackDrop1_distalRed_y * RED;
                closeToIntake = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark_x = spikeMark2_x_Blue_Distal;
                spikeMark_y = spikeMark2_y_Blue_Distal * RED;
                spikeMark_heading = 0; // initial 185  //5
                detectedBackDrop_x = BackDrop2_distalRed_x;
                detectedBackDrop_y = BackDrop2_distalRed_y * RED;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark_x = spikeMark1_x_Blue_Distal;
                spikeMark_y = spikeMark1_y_Blue_Distal * RED;
                spikeMark_heading = 0;
                detectedBackDrop_x = BackDrop3_distalRed_x;
                detectedBackDrop_y = BackDrop3_distalRed_y * RED;
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

        sleep(300);
        drive.followTrajectory(moveToDetectedBackDrop);
        sleep(200);
    }

    public void DistalBackDropDump(){
        if(isStopRequested()) return;
        Trajectory fromDistalToBack=drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))//remember to change y
                .back(5)
                .build();
        Trajectory fromDistalToIntermediate=drive.trajectoryBuilder(new Pose2d(spikeMark_x-5,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .strafeTo(new Vector2d(spikeMark_x-5,intermediate_y*side_color))
                .build();

//         Trajectory closeToFar = drive.trajectoryBuilder(new Pose2d(spikeMark_x - 5, intermediate_y * side_color, Math.toRadians(spikeMark_heading)))
//                     .forward(5)
//                     .build();


        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(spikeMark_x-5,intermediate_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(29.39,intermediate_y*side_color))
                .build();
        /*old version
        Trajectory fromProximalToSpikeMark = drive.trajectoryBuilder(new Pose2d(29.39,intermediate_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(29.39,detectedBackDrop_y*side_color))
                .build();

        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,detectedBackDrop_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();
         */
        Trajectory fromProximalToSpikeMark = drive.trajectoryBuilder(new Pose2d(29.39,intermediate_y*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(29.39,detectedBackDrop_y*side_color))
                .build();

        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,intermediate_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();
        drive.followTrajectory(fromDistalToBack);
        drive.followTrajectory(fromDistalToIntermediate);
        if(closeToIntake){
            //drive.followTrajectory(closeToFar);
            farPos_x = spikeMark_x;
            farPos_y=intermediate_y*side_color;
            farPos_heading=Math.toRadians(180);
        }else{
            farPos_x=spikeMark_x-5;
            farPos_y=intermediate_y*side_color;
            farPos_heading=Math.toRadians(180);
        }
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(fromIntermediateToProximal);
        //drive.followTrajectory(fromProximalToSpikeMark);

        sleep(300);
        drive.followTrajectory(fromProximalToBackdrop);

        sleep(400);
        sleep(500);
        sleep(300);

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
                .lineTo(new Vector2d(ec_final_backDrop_x,ec_final_backDrop_y*side_color))
                .build();

        drive.followTrajectory(moveToSide);
        drive.followTrajectory(moveToDistal);
        drive.followTrajectory(moveToPixel);
        //drive.followTrajectory(moveToIntake);

        sleep(200);

        //intake2();
        intake22();//test for new intake with no arm expand//
        drive.followTrajectory(fromIntakeToIntermediate);
        drive.followTrajectory(fromIntermediateToProximal);

        sleep(200);
        sleep(300);
        //drive.followTrajectory(fromProximalToSpikeMark);
        drive.followTrajectory(fromSpikeMarkToBackdrop); // need to change


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
