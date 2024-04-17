//package org.firstinspires.ftc.teamcode.FTC16093.auto;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Autonomous
//@Config
//public class AutoRedNear extends AutoMaster {
//    @Override
//    public void runOpMode() throws InterruptedException{
//        startSide = PROXIMAL;
//        side_color = RED;
//        drop_side = RIGHT;
//
//        initHardware();
//        if(auto_mode==1){
//            if(true){//2+0 不用分中间还是边上
//                if(drop_side==LEFT){//2+0 走中间 放左边
//                    spikeMarkDump();
//                    putOnSpikeMark();
//                    backDropDump();
//                    upper.setArmPosition(2077);
//                    sleep(500);
//                    upper.grab2_open();
//                    sleep(300);
//                    setUpAuto();
//                    parking(park);
//                }else if(drop_side==RIGHT){//2+0 走中间 放右边
//                    BackDrop_RedLeft_y = -28;  // y: left -26.5 right -28
//                    BackDrop_RedCenter_y = -35; // y: left -32.5 right -35
//                    BackDrop_RedRight_y = -41; // y: left -38.5 right -41
//                    spikeMarkDump();
//                    putOnSpikeMark();
//                    backDropDump();
//                    upper.setArmPosition(2077);
//                    sleep(500);
//                    upper.grab2_open();
//                    sleep(300);
//                    setUpAuto();
//                    parking_2plus0(park);
//                }
//            }
//        }else if(auto_mode==2){
//            if(auto_route==1){
//                if(drop_side==LEFT){//2+2 走中间 放左边
//                    spikeMarkDump();
//                    putOnSpikeMark();
//                    backDropDump();
//                    upper.setArmPosition(2077);
//                    sleep(500);
//                    upper.grab2_open();
//                    sleep(300);
//                    setUpAuto();
//                    ecByCenter_farCenter();
//                    drop_upward_grab1();
//                    setUpAuto();
//                    sleep(1000);
//                    parking(park);
//                }else if(drop_side==RIGHT){//2+2 走中间 放右边
//                    spikeMarkDump();
//                    putOnSpikeMark();
//                    backDropDump();
//
//                    upper.setArmPosition(2077);
//                    sleep(500);
//                    upper.grab2_open();
//                    sleep(300);
//                    setUpAuto();
//
//                    ecByCenter_farCenter();
//                    drop_upward_grab1();
//
//                    setUpAuto();
//                    sleep(1000);
//                    parking(park);
//                }
//            }else if(auto_route==2){
//                if(drop_side==LEFT){//2+2 走边上 放左边
//                    spikeMarkDump();
//                    putOnSpikeMark();
//                    backDropDump();
//
//                    upper.setArmPosition(2077);
//                    sleep(500);
//                    upper.grab2_open();
//                    sleep(300);
//                    setUpAuto();
//                    ec_lowFar_edgeSpline_blue();
//                    parking(park);
//                }else if(drop_side==RIGHT){//2+2 走边上 放右边
//                    spikeMarkDump();
//                    putOnSpikeMark();
//                    backDropDump();
//
//                    upper.setArmPosition(2077);
//                    sleep(500);
//                    upper.grab2_open();
//                    sleep(300);
//                    setUpAuto();
//                    ec_lowFar_edgeSpline_blue();
//                    parking(park);
//                }
//            }
//
//        }
//        if(back_start){
//            backTo_StartPos();
//        }
//        sleep(114514);
//    }
//
//}
//
