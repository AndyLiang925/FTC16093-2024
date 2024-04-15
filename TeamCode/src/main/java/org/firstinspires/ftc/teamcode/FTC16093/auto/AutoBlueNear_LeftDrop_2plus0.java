//package org.firstinspires.ftc.teamcode.FTC16093.auto;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Autonomous (group = "AutoBlueNear")
//@Config
//public class AutoBlueNear_LeftDrop_2plus0 extends AutoMaster {
//    @Override
//    public void runOpMode() throws InterruptedException{
//        BackDrop_blueRight_y = 22.2;
//        BackDrop_blueCenter_y = 28.3;
//        BackDrop_blueLeft_y = 33.9;
//        startSide = PROXIMAL;
//        side_color = BLUE;
//        initHardware();
//
//        spikeMarkDump();
//        putOnSpikeMark();
//
//        backDropDump();
//
//        upper.setArmPosition(2100);
//        sleep(500);
//        upper.grab2_open();
//        sleep(300);
//        setUpAuto();
//
//        //extraCredit();
//        ecByCenter_farCenter();
//        //ec_lowFar_edgeSpline_blue()
//        raiseArm_slow(armPosUpward);
//        putOnBackDrop_grab2();
//        raiseArm_slow(armPosUpward-30);
//        sleep(500);
//        upper.setArmPosition_slow(armPosUpward-120);
//        setUpAuto();
//        backTo_StartPos();
////        sleep(1000);
////        parking(1);
//    }
//
//}
