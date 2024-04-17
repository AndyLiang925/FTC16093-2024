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

    protected int selected_line=1;
    protected int auto_mode=1;//1:2+0 1:2+1 2:2+2 2:2+3
    protected int auto_route=1;//1:center 2:edge
    protected int park=1;//1:outer 2:inner 3:no parking

    boolean y_pressed=false;
    boolean a_pressed=false;
    boolean xb_pressed=false;

    public static final int PROXIMAL = 1;
    public static final int DISTAL = -1;
    public static final int LEFT = 1;
    public static final int RIGHT = -1;
    public static boolean DEBUG = true;
    public static int startTimeDelay = -1, cycleDelay = -1;
    public static double DESIRED_DISTANCE=0.4;

    public static final int RED = -1;
    public static final int BLUE = 1;

    protected int startSide;
    protected int side_color;
    protected int drop_side;
    protected boolean back_start=false;
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
    public static double spikeMark_RedLeft_x=13,spikeMark_RedLeft_y= -24;
    public static double spikeMark_RedCenter_x=20.5,spikeMark_RedCenter_y= -17; //-18
    public static double spikeMark_RedRight_x = 36 ,spikeMark_RedRight_y = -22;

    public static double spikeMark_blueLeft_x = 35, spikeMark_blueLeft_y = 26; // y initial 30
    public static double spikeMark_blueCenter_x = 26, spikeMark_blueCenter_y = 21;
    public static double spikeMark_blueRight_x = 13,spikeMark_blueRight_y = 30; // y initial 30


    public static double spikeMarkCenter_x,spikeMarkCenter_y;
    public static double spikeMarkCenterProximal_x = 20,spikeMarkCenterProximal_y = 33; // x initial 45
    public static double spikeMarkCenterDistal_x = -45,spikeMarkCenterDistal_y = 30; //push

    public static double spikeMark_blue_DistalLeft_x = -36, spikeMark_blue_DistalLeft_y = 25; //27
    public static double spikeMark_blue_DistalCenter_x = -43, spikeMark_blue_DistalCenter_y = 15;
    public static double spikeMark_blue_DistalRight_x = -37, spikeMark_blue_DistalRight_y = 30;

    public static double spikeMark_Red_DistalLeft_x = -35, spikeMark_Red_DistalLeft_y = -20;
    public static double spikeMark_Red_DistalCenter_x = -40, spikeMark_Red_DistalCenter_y = -8;
    public static double spikeMark_Red_DistalRight_x = -38, spikeMark_Red_DistalRight_y = -26;

    public static double BackDrop_RedLeft_x = 49, BackDrop_RedLeft_y = -26.5;  // y: left -26.5 right -28
    public static double BackDrop_RedCenter_x = 49, BackDrop_RedCenter_y = -32.5; // y: left -32.5 right -35
    public static double BackDrop_RedRight_x = 49, BackDrop_RedRight_y = -38.5; // y: left -38.5 right -41

    public static double BackDrop_blueRight_x = 48.5,BackDrop_blueRight_y = 26; // y: left 24.3 right 22.2
    public static double BackDrop_blueCenter_x = 48.5, BackDrop_blueCenter_y = 32; // y: left 29.5 right 28.3
    public static double BackDrop_blueLeft_x =48.5,BackDrop_blueLeft_y = 36; // y: left 36.2 right 33.9

    public static double backDrop_blue_distalRight_x = 48.5,backDrop_blue_distalRight_y = 23; // y: left 24.5 right 23
    public static double backDrop_blue_distalCenter_x = 48.5,backDrop_blue_distalCenter_y = 28; // y: left: 28 right 27
    public static double backDrop_blue_distalLeft_x = 48.5,backDrop_blue_distalLeft_y = 34; // y: left 34 right 32.7

    public static double backDrop_red_distalLeft_x = 49,backDrop_red_distalLeft_y = -27;
    public static double backDrop_red_distalCenter_x = 49,backDrop_red_distalCenter_y = -34;
    public static double backDrop_red_distalRight_x = 49,backDrop_red_distalRight_y = -42.5;

    public static double detectedBackDrop_x,detectedBackDrop_y, detectedBackDrop_heading=180;

    public static double ec_backDrop_blue_distalLeft_y = 24;
    public static double intake_far_x = -43,intake_farCenter_y = -6.5;
    public static double intake_far_redCenter_y = -7.2, intake_far_blueCenter_y = 5;

    public static double intake_blue_near_x = -57, intake_blue_left_near_y = 9;
    public static double intake_blue_center_near_y = 18;

    public static double intake_blue_right_oblique_x = - 43, intake_blue_right_oblique_y = 33,intake_oblique_heading_grab1= 180;
    public static double intake_distal_red_y = -4.5;
    public static double park_x = 43, park_inside=56 , park_outside = 6;
    public static double intake_x = -43, intake_y = 30.5 ;
    public static double intake_oblique_blue_x = -46, intake_oblique_blue_y = 11;
    public static double intake_near_x = -58, intake_distal_y = 9, intake_red_near_y = -5;
    public static double intake_distal_near_y = 9;
    public static double intake_medi_x = -38; // intial -35
    public static double intake_center_far_x= -43, intake_center_far_y= 34;

    public static double intake_medi_y=5, ec_intake_medi_y=6;
    public static double ec_side_middle_x=-45;
    public static double ec_side_intake_x=-55;
    public static double ec_side_intake_y=15;
    public static double ec_side_intake_turnangle=60;

    public static double edge_medi_y = 57;
    public static double ec_backDrop_x, ec_backDrop_y;
    public static int armPosUpward=2150;

    public static boolean closeToIntake = false, kickProp = false;

    public static double forwardDistance=3;
    public superstructure upper;
    public static int armPos = 280,armPos_near1 = 135, armPos_near_low = 50, armPos_far_low = 230, armPos_delta = 70;//175 armposnear1=140
    public static int wait_time = 0,sleep_2=1000,sleep_3=1000;

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
            //telemetry.update();
            long time = System.currentTimeMillis();
            //telemetry.update();
            sleep(15);
            //while (System.currentTimeMillis() - time < 100 && opModeInInit()) idle();
            if (isStopRequested()) throw new InterruptedException();
            telemetry.addLine("------------------------------------------------");
            telemetry.addLine("|"+(selected_line==1?">":"  ")+"自动跑几趟:"+(startSide==PROXIMAL?(auto_mode==1?"2+0":"2+2"):(auto_mode==1?"2+1":"2+3"))+"|");
            telemetry.addLine("|"+(selected_line==2?">":"  ")+"自动路径:"+(auto_route==1?"中间跑":"边上跑")+"|");
            telemetry.addLine("|"+(selected_line==3?">":"  ")+"放那边:"+(drop_side==LEFT?"左":"右")+"|");
            telemetry.addLine("|"+(selected_line==4?">":"  ")+"停靠:"+(park==3?"停板前":(park==1?"板外侧":"板内侧"))+"|");
            telemetry.addLine("|"+(selected_line==5?">":"  ")+"[比赛不要开启]自动回到初始位置:"+(back_start?"开启":"关闭")+"|");
            telemetry.addLine("------------------------------------------------");
            telemetry.update();
            if(gamepad1.y&&!y_pressed){
                selected_line=Math.max(1,selected_line-1);
                y_pressed=true;
            }
            if(gamepad1.a&&!a_pressed){
                selected_line=Math.min(5,selected_line+1);
                a_pressed=true;
            }
            if((gamepad1.b||gamepad1.x)&&!xb_pressed){
                auto_mode=selected_line==1?(auto_mode==1?2:1):auto_mode;
                auto_route=selected_line==2?(auto_route==1?2:1):auto_route;
                drop_side=selected_line==3?(drop_side==LEFT?RIGHT:LEFT):drop_side;
                park=selected_line==4?(gamepad1.b?Math.max(1,park-1):Math.min(3,park+1)):park;
                back_start=selected_line==5?!back_start:back_start;
                xb_pressed=true;
            }
            if(auto_mode==2){
                park=3;
            }
            y_pressed=(y_pressed?gamepad1.y:y_pressed);
            a_pressed=(a_pressed?gamepad1.a:a_pressed);
            xb_pressed=(xb_pressed?gamepad1.x||gamepad1.b:xb_pressed);
        }
        waitForStart();

        if(startSide==PROXIMAL) {
            spikeMarkCenter_x=spikeMarkCenterProximal_x;
            spikeMarkCenter_y=spikeMarkCenterProximal_y;
            if (drop_side == LEFT){
                BackDrop_blueRight_y = 26;
                BackDrop_blueCenter_y = 32;
                BackDrop_blueLeft_y = 38;

                BackDrop_RedLeft_y = -26.5;
                BackDrop_RedCenter_y = -32.5;
                BackDrop_RedRight_y = -38.5;
            } else if (drop_side == RIGHT) {
                BackDrop_blueRight_y = 22.2;
                BackDrop_blueCenter_y = 28.6;
                BackDrop_blueLeft_y = 35.2;

                BackDrop_RedLeft_y = -28.5;
                BackDrop_RedCenter_y = -35;
                BackDrop_RedRight_y = -41;
            }
            if(side_color == BLUE){
                intake_farCenter_y = intake_far_blueCenter_y;//
            } else if (side_color == RED) {
                intake_farCenter_y = intake_far_redCenter_y;
            }

            if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE) {
                DesiredTagId = 1;
                spikeMark_x=spikeMark_blueLeft_x ;
                spikeMark_y=spikeMark_blueLeft_y;
                spikeMark_heading=180;
                detectedBackDrop_x= BackDrop_blueLeft_x;
                detectedBackDrop_y= BackDrop_blueLeft_y;

                ec_backDrop_x = BackDrop_blueRight_x-2;
                ec_backDrop_y = 29;
                kickProp = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark_x=spikeMark_blueCenter_x;
                spikeMark_y=spikeMark_blueCenter_y;
                spikeMark_heading=180;
                detectedBackDrop_x= BackDrop_blueCenter_x;
                detectedBackDrop_y= BackDrop_blueCenter_y;
                ec_backDrop_x = BackDrop_blueRight_x-2;
                ec_backDrop_y = BackDrop_blueRight_y;
                kickProp = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark_x=spikeMark_blueRight_x;
                spikeMark_y=spikeMark_blueRight_y;
                spikeMark_heading=180;
                detectedBackDrop_x= BackDrop_blueRight_x;
                detectedBackDrop_y= BackDrop_blueRight_y;
                ec_backDrop_x = BackDrop_blueLeft_x-2;
                ec_backDrop_y = BackDrop_blueLeft_y;
                kickProp = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark_x = spikeMark_RedLeft_x  ;
                spikeMark_y = spikeMark_RedLeft_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = BackDrop_RedLeft_x;
                detectedBackDrop_y = BackDrop_RedLeft_y ;

                ec_backDrop_x = BackDrop_RedRight_x-2;
                ec_backDrop_y = -34;
                kickProp = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark_x = spikeMark_RedCenter_x ;
                spikeMark_y = spikeMark_RedCenter_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = BackDrop_RedCenter_x;
                detectedBackDrop_y = BackDrop_RedCenter_y ;

                ec_backDrop_x = BackDrop_RedLeft_x-2;
                ec_backDrop_y = BackDrop_RedLeft_y;
                kickProp = false;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark_x = spikeMark_RedRight_x ;
                spikeMark_y = spikeMark_RedRight_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = BackDrop_RedRight_x;
                detectedBackDrop_y = BackDrop_RedRight_y ;

                ec_backDrop_x = BackDrop_RedLeft_x-2;
                ec_backDrop_y = BackDrop_RedLeft_y;
                kickProp = false;
            }
        }
        if(startSide==DISTAL){
            spikeMarkCenter_x = spikeMarkCenterDistal_x;
            spikeMarkCenter_y = spikeMarkCenterDistal_y;
            if (drop_side == LEFT){
                backDrop_blue_distalRight_y = 24.5; // y: left 23 right 24.5
                backDrop_blue_distalCenter_y = 28; // y: left: 28 right 27
                backDrop_blue_distalLeft_y = 34; // y: left 34 right 32.7

                backDrop_red_distalLeft_y = -25; //猜的
                backDrop_red_distalCenter_y = -34;
                backDrop_red_distalRight_y = -40;
            } else if (drop_side == RIGHT) {
                backDrop_blue_distalRight_y = 23;
                backDrop_blue_distalCenter_y = 27;
                backDrop_blue_distalLeft_y = 32.7;

                backDrop_red_distalLeft_y = -27;
                backDrop_red_distalCenter_y = -36;
                backDrop_red_distalRight_y = -42;
            }
            if(side_color == BLUE){
                intake_distal_y = intake_blue_left_near_y;
                intake_farCenter_y = intake_far_blueCenter_y;  
            } else if (side_color == RED) {
                intake_distal_y = intake_distal_red_y;
                intake_farCenter_y = intake_far_redCenter_y;
            }

            if(startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == BLUE){
                DesiredTagId=1;
                spikeMark_x=spikeMark_blue_DistalLeft_x;
                spikeMark_y=spikeMark_blue_DistalLeft_y;
                spikeMark_heading= 0;
                detectedBackDrop_x= backDrop_blue_distalLeft_x;
                detectedBackDrop_y= backDrop_blue_distalLeft_y;
                ec_backDrop_x = backDrop_blue_distalRight_x;
                ec_backDrop_y = ec_backDrop_blue_distalLeft_y;
                kickProp = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == BLUE) {
                DesiredTagId = 2;
                spikeMark_x=spikeMark_blue_DistalCenter_x;
                spikeMark_y=spikeMark_blue_DistalCenter_y;
                spikeMark_heading= 0 ;
                detectedBackDrop_x= backDrop_blue_distalCenter_x;
                detectedBackDrop_y= backDrop_blue_distalCenter_y;
                ec_backDrop_x = backDrop_blue_distalRight_x;
                ec_backDrop_y = backDrop_blue_distalRight_y;
                kickProp = false;
                closeToIntake = false;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == BLUE) {
                DesiredTagId = 3;
                spikeMark_x = spikeMark_blue_DistalRight_x;
                spikeMark_y = spikeMark_blue_DistalRight_y;
                spikeMark_heading = 180;
                detectedBackDrop_x = backDrop_blue_distalRight_x;
                detectedBackDrop_y = backDrop_blue_distalRight_y;
                ec_backDrop_x = backDrop_blue_distalLeft_x;
                ec_backDrop_y = backDrop_blue_distalLeft_y;
                closeToIntake = true;
                kickProp = false;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.LEFT && side_color == RED) {
                DesiredTagId = 4;
                spikeMark_x = spikeMark_Red_DistalLeft_x;
                spikeMark_y = spikeMark_Red_DistalLeft_y ;
                spikeMark_heading = 180;
                detectedBackDrop_x = backDrop_red_distalLeft_x;
                detectedBackDrop_y = backDrop_red_distalLeft_y;
                ec_backDrop_x = backDrop_red_distalRight_x;
                ec_backDrop_y = backDrop_red_distalRight_y;
                closeToIntake = true;
                kickProp = true;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED) {
                DesiredTagId = 5;
                spikeMark_x = spikeMark_Red_DistalCenter_x;
                spikeMark_y = spikeMark_Red_DistalCenter_y;
                spikeMark_heading = -90;
                detectedBackDrop_x = backDrop_red_distalCenter_x;
                detectedBackDrop_y = backDrop_red_distalCenter_y;
                ec_backDrop_x = backDrop_red_distalLeft_x;
                ec_backDrop_y = backDrop_red_distalLeft_y;
                kickProp = false;
            } else if (startingPos == CenterStageVisionProcessor.StartingPosition.RIGHT && side_color == RED) {
                DesiredTagId = 6;
                spikeMark_x = spikeMark_Red_DistalRight_x;
                spikeMark_y = spikeMark_Red_DistalRight_y ;
                spikeMark_heading = 0;
                detectedBackDrop_x = backDrop_red_distalRight_x;
                detectedBackDrop_y = backDrop_red_distalRight_y;
                ec_backDrop_x = backDrop_red_distalLeft_x;
                ec_backDrop_y = backDrop_red_distalLeft_y;
                kickProp = true;
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
    public void spikeMarkDump(){
        if (isStopRequested()) return;
        Trajectory moveToCenter;
        moveToCenter = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(spikeMarkCenter_x,spikeMarkCenter_y*side_color,Math.toRadians(spikeMark_heading)))
                .build();

        Trajectory moveToSpikeMark;
        moveToSpikeMark = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(spikeMark_x, spikeMark_y,Math.toRadians(spikeMark_heading)))
                .build();

        if(kickProp){
            drive.followTrajectory(moveToCenter);
            upper.wristDown();

            moveToSpikeMark = drive.trajectoryBuilder(new Pose2d(spikeMarkCenter_x,spikeMarkCenter_y*side_color,Math.toRadians(spikeMark_heading))) //can change to start pos
                    .lineToLinearHeading(new Pose2d(spikeMark_x, spikeMark_y,Math.toRadians(spikeMark_heading)))
                    .build();
        }
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
        Trajectory moveToDetectedBackDrop=drive.trajectoryBuilder(new Pose2d(spikeMark_x, spikeMark_y,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(detectedBackDrop_x-7,detectedBackDrop_y,Math.toRadians(180)))
                .build();
        Trajectory moveToDetectedBackDrop_closer=drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x-7,detectedBackDrop_y,Math.toRadians(spikeMark_heading)))
                .lineToLinearHeading(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(180)))
                .build();
        upper.setArmPosition(1400);
        drive.followTrajectory(moveToDetectedBackDrop);
        drive.followTrajectory(moveToDetectedBackDrop_closer);
    }
    public void distal_intake_center(){
        if(isStopRequested()) return;
        Trajectory back = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .back(forwardDistance)
                .build();
        Trajectory toMediByLine = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .lineToLinearHeading(new Pose2d(intake_medi_x,intake_distal_y,Math.toRadians(180)))
                .build();
        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(intake_medi_x,intake_distal_y,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_near_x,intake_distal_y))
                .build();
        if (startSide == DISTAL && startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED){
            drive.followTrajectory(back);
            toMediByLine = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y+3,Math.toRadians(spikeMark_heading)))
                    .lineToLinearHeading(new Pose2d(intake_medi_x,intake_distal_y,Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectory(toMediByLine);
        upper.autoGrabPrepare(armPos_near1);
        drive.followTrajectory(moveToIntake);
        upper.autoGrabFinish();
    }
    public void distal_backDropDump_center(){
        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_near_x,intake_distal_y,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(29.39,intake_medi_y*side_color))
                .build();
        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,intake_medi_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();
        drive.followTrajectory(fromIntermediateToProximal);
        upper.setArmPosition(1780);
        drive.followTrajectory(fromProximalToBackdrop);
    }
    public void DistalBackDropDump(){
        if(isStopRequested()) return;
        Trajectory back = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .back(forwardDistance)
                .build();

        Trajectory toMediByLine = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y,Math.toRadians(spikeMark_heading)))
                .lineToLinearHeading(new Pose2d(intake_medi_x,intake_distal_y,Math.toRadians(180)))
                .build();

        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(intake_medi_x,intake_distal_y,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(intake_near_x,intake_distal_y,Math.toRadians(180)))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_near_x,intake_distal_y,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(29.39,intake_medi_y*side_color))
                .build();
        Trajectory fromProximalToBackdrop = drive.trajectoryBuilder(new Pose2d(29.39,intake_medi_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(detectedBackDrop_x,detectedBackDrop_y))
                .build();

        if (startSide == DISTAL && startingPos == CenterStageVisionProcessor.StartingPosition.CENTER && side_color == RED){
            drive.followTrajectory(back);
            toMediByLine = drive.trajectoryBuilder(new Pose2d(spikeMark_x,spikeMark_y+3,Math.toRadians(spikeMark_heading)))
                    .lineToLinearHeading(new Pose2d(intake_medi_x,intake_distal_y,Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectory(toMediByLine);
        upper.autoGrabPrepare(armPos_near1);
        drive.followTrajectory(moveToIntake);
        upper.autoGrabFinish();
        sleep(wait_time);
        drive.followTrajectory(fromIntermediateToProximal);
        upper.setArmPosition(1780);
        drive.followTrajectory(fromProximalToBackdrop);
    }

    public void ec_lowFar_edgeSpline_blue(){
        Trajectory moveToMiddle = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(180)))
                .splineTo(new Vector2d(30, 54*side_color), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-37,54*side_color, Math.toRadians(180.00)))
                .build();

        Trajectory moveToIntake = drive.trajectoryBuilder(new Pose2d(-37, 54*side_color), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(intake_blue_right_oblique_x, intake_blue_right_oblique_y,Math.toRadians(180)))
                .build();

        Trajectory moveToBack= drive.trajectoryBuilder(new Pose2d(intake_blue_right_oblique_x,intake_blue_right_oblique_y,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-35,53*side_color,Math.toRadians(180)))
                .build();

        Trajectory back= drive.trajectoryBuilder(new Pose2d(-35,53*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40,53*side_color))
                .build();

        Trajectory drop= drive.trajectoryBuilder(new Pose2d(40,53*side_color,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(ec_backDrop_x-1,ec_backDrop_y))
                .build();

        drive.followTrajectory(moveToMiddle);
        sleep(300);
        drive.followTrajectory(moveToIntake);
        intake2_lowfar_grab1();
        drive.followTrajectory(moveToBack);
        drive.followTrajectory(back);
        upper.setArmPosition(1600);
        upper.wrist_to_upward();
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
        if(side==3){
            return;
        }
        if(isStopRequested()) return;
        Trajectory moveToPark = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,ec_backDrop_y,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(park_x,park_inside*side_color,Math.toRadians(-90*side_color)))
                .build();
        Trajectory moveToBack=drive.trajectoryBuilder(new Pose2d(park_x,park_inside*side_color,Math.toRadians(-90*side_color)))
                .lineToConstantHeading(new Vector2d(park_x+15,park_inside*side_color))
                .build();
        if (side==2){
            moveToPark = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,ec_backDrop_y,Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(park_x,park_outside*side_color,Math.toRadians(-90*side_color)))
                    .build();
            moveToBack=drive.trajectoryBuilder(new Pose2d(park_x,park_outside*side_color,Math.toRadians(-90*side_color)))
                    .lineToConstantHeading(new Vector2d(park_x+15,park_outside*side_color))
                    .build();
        }
        drive.followTrajectory(moveToPark);
        drive.followTrajectory(moveToBack);
    }
    public void parking_2plus0(int side){
        if(isStopRequested()) return;
        Trajectory moveToPark = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(park_x,park_inside*side_color,Math.toRadians(-90*side_color)))
                .build();
        Trajectory moveToBack=drive.trajectoryBuilder(new Pose2d(park_x,park_inside*side_color,Math.toRadians(-90*side_color)))
                .lineToConstantHeading(new Vector2d(park_x+15,park_inside*side_color))
                .build();
        if (side==2){
            moveToPark = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(park_x,park_outside*side_color,Math.toRadians(-90*side_color)))
                    .build();
            moveToBack=drive.trajectoryBuilder(new Pose2d(park_x,park_outside*side_color,Math.toRadians(-90*side_color)))
                    .lineToConstantHeading(new Vector2d(park_x+15,park_outside*side_color))
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
        upper.setArmLength(-20);
        sleep(200);
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
        sleep(200);
        upper.wrist_to_middle();
        //upper.setArmPosition(1890);
        sleep(300);
    }
    public void putOnBackDrop_grab2(){
        upper.grab2_open();
        sleep(200);
        //upper.grab2_close();
    }
    public void raiseArm_slow(int pos){
        upper.setArmPosition_slow(pos);
        sleep(sleep_2);
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
        sleep(700);
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
        sleep(700);
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
//        upper.wristDown();
//        sleep(100);
        upper.grab1_open();
        sleep(200);
        //upper.wrist_to_middle();
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
                .splineTo(new Vector2d(10,intake_distal_y),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_x, intake_distal_y))
                .build();

        Trajectory moveForward = drive.trajectoryBuilder(new Pose2d(intake_x,intake_distal_y,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(intake_near_x-2,intake_distal_y))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_near_x-2,intake_distal_y,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(35,intake_medi_y*side_color))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(35, intake_medi_y*side_color,Math.toRadians(180)))
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
                .splineTo(new Vector2d(10,intake_distal_y),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_x, intake_distal_y))
                .build();
        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_x,intake_distal_y,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(10,intake_medi_y*side_color))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(30, intake_medi_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();

        drive.followTrajectory(intake);
        intake2_byGrab1();
        drive.followTrajectory(fromIntermediateToProximal);
        drive.followTrajectory(drop);
    }

    public void ecByCenter_farCenter(){
        Trajectory moveToPixel = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(30,intake_medi_y*side_color),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(intake_medi_x,intake_medi_y*side_color,Math.toRadians(180)))
                .build();

        Trajectory intake = drive.trajectoryBuilder(new Pose2d(intake_medi_x,intake_medi_y*side_color, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(intake_far_x,intake_farCenter_y,Math.toRadians(180)))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_far_x,intake_farCenter_y,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(30,intake_medi_y*side_color))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(30, intake_medi_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();

        drive.followTrajectory(moveToPixel);
        drive.followTrajectory(intake);
        if(side_color == BLUE){
            intake2_lowFar_grab2();
        }else{
            intake2_lowfar_grab1();
        }

        drive.followTrajectory(fromIntermediateToProximal);
        upper.wrist_to_upward();
        upper.setArmPosition(1800);
        drive.followTrajectory(drop);
    }
    public void ec_lowFar_blueMiddle(){
        Trajectory moveToPixel = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(30,intake_medi_y*side_color),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_medi_x, intake_medi_y*side_color))
                .build();

        Trajectory intake = drive.trajectoryBuilder(new Pose2d(intake_medi_x,intake_medi_y*side_color, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(intake_oblique_blue_x,intake_oblique_blue_y*side_color,Math.toRadians(180-side_color*15)))
                .build();

        Trajectory moveToMiddle = drive.trajectoryBuilder(new Pose2d(intake_oblique_blue_x,intake_oblique_blue_y*side_color,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(intake_oblique_blue_x,intake_medi_y*side_color))
                .build();
        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_oblique_blue_x,intake_medi_y*side_color,Math.toRadians(180))) //!metion
                .lineToConstantHeading(new Vector2d(30,intake_medi_y*side_color))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(30, intake_medi_y*side_color,Math.toRadians(180)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();

        drive.followTrajectory(moveToPixel);
        drive.followTrajectory(intake);
        if(side_color == BLUE){
            intake2_lowFar_grab2();
        }else{
            intake2_lowfar_grab1();
        }
        drive.followTrajectory(moveToMiddle);
        drive.followTrajectory(fromIntermediateToProximal);
        upper.wrist_to_upward();
        upper.setArmPosition(1800);
        drive.followTrajectory(drop);
    }

    public void ecBySide(){
        Trajectory moveToStart = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(startPos_x, startPos_y-side_color*3, Math.toRadians(180.00)))
                .build();
        Trajectory fromStartToMiddle = drive.trajectoryBuilder(new Pose2d(startPos_x, startPos_y-side_color*3, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(ec_side_middle_x, startPos_y-side_color*3, Math.toRadians(180.00)))
                .build();
        Trajectory fromMiddleToIntake = drive.trajectoryBuilder(new Pose2d(ec_side_middle_x, startPos_y-side_color*3, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(ec_side_intake_x, ec_side_intake_y, Math.toRadians(180.00+side_color*ec_side_intake_turnangle)))
                .build();
        Trajectory drop = drive.trajectoryBuilder(new Pose2d(startPos_x, startPos_y-side_color*3,Math.toRadians(180)))
                .lineTo(new Vector2d(ec_backDrop_x,ec_backDrop_y))
                .build();
        Trajectory fromMiddleToStart = drive.trajectoryBuilder(new Pose2d(ec_side_middle_x, startPos_y-side_color*3, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(startPos_x, startPos_y-side_color*3, Math.toRadians(180.00)))
                .build();
        Trajectory fromIntakeToMiddle = drive.trajectoryBuilder(new Pose2d(ec_side_intake_x, ec_side_intake_y, Math.toRadians(180.00+side_color*ec_side_intake_turnangle)))
                .lineToLinearHeading(new Pose2d(ec_side_middle_x, startPos_y-side_color*3, Math.toRadians(180.00)))
                .build();
        drive.followTrajectory(moveToStart);
        drive.followTrajectory(fromStartToMiddle);
        drive.followTrajectory(fromMiddleToIntake);
        intake2_lowFar_grab2();
        drive.followTrajectory(fromIntakeToMiddle);
        drive.followTrajectory(fromMiddleToStart);
        upper.wrist_to_upward();
        upper.setArmPosition(1600);
        drive.followTrajectory(drop);
    }

    public void ec_far_putOnGround(){
        Trajectory intake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(30,intake_farCenter_y),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_far_x, intake_farCenter_y))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_far_x,intake_farCenter_y,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(55,intake_farCenter_y,Math.toRadians(135*side_color)))
                .build();

        drive.followTrajectory(intake);
        intake2_lowfar_grab1();
        drive.followTrajectory(fromIntermediateToProximal);
        dropGround_low();
    }
    public void ec_distal_farCenter_grab1(){
        Trajectory intake = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x, detectedBackDrop_y, Math.toRadians(180.00)))
                .splineTo(new Vector2d(30,intake_distal_y),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(intake_far_x, intake_farCenter_y))
                .build();

        Trajectory fromIntermediateToProximal = drive.trajectoryBuilder(new Pose2d(intake_far_x,intake_farCenter_y,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(55,intake_farCenter_y,Math.toRadians(135*side_color)))
                .build();

        drive.followTrajectory(intake);
        intake2_lowfar_grab1();
        drive.followTrajectory(fromIntermediateToProximal);
        dropGround_low();
    }

    public void backTo_StartPos(){
        Trajectory startpos = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,detectedBackDrop_y,Math.toRadians(180)))
                .lineToLinearHeading(startPos)
                .build();
        drive.followTrajectory(startpos);
    }
    public void backTo_distalStartPos(){
        Trajectory start = drive.trajectoryBuilder(new Pose2d(detectedBackDrop_x,ec_backDrop_y,Math.toRadians(180)))
                .splineTo(new Vector2d(30,intake_medi_y*side_color),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(45,intake_medi_y*side_color))
                .build();
        Trajectory second = drive.trajectoryBuilder(new Pose2d(45,intake_medi_y*side_color,Math.toRadians(180)))
                .lineToLinearHeading(startPos)
                .build();
        drive.followTrajectory(start);
        drive.followTrajectory(second);
    }
    public void drop_upward_grab1(){
        raiseArm_slow(armPosUpward);
        sleep(300);
        putOnBackDrop_grab1();
        sleep(200);
        upper.wrist_to_upward_drop();
        sleep(200);
        raiseArm_slow(armPosUpward-armPos_delta);
        //
        upper.set_wrist_pos(0.28);
        sleep(150);
        upper.set_wrist_pos(0.3);
        sleep(150);
        upper.set_wrist_pos(0.28);
        sleep(150);
        upper.set_wrist_pos(0.3);
        sleep(150);
        upper.set_wrist_pos(0.28);
        sleep(150);
        upper.set_wrist_pos(0.3);
        upper.setArmPosition_slow(armPosUpward-120);

    }
    public void drop_upward_grab2(){
        raiseArm_slow(armPosUpward);
        sleep(800);
        putOnBackDrop_grab2();
        sleep(300);
        upper.wrist_to_upward_drop();
        sleep(200);
        raiseArm_slow(armPosUpward-armPos_delta);

        upper.set_wrist_pos(0.28);
        sleep(150);
        upper.set_wrist_pos(0.3);
        sleep(150);
        upper.set_wrist_pos(0.28);
        sleep(150);
        upper.set_wrist_pos(0.3);
        sleep(150);
        upper.set_wrist_pos(0.28);
        sleep(150);
        upper.set_wrist_pos(0.3);
        sleep(500);
        upper.setArmPosition_slow(armPosUpward-120);
    }
    public void distal_edgeBack(){

    }
}
