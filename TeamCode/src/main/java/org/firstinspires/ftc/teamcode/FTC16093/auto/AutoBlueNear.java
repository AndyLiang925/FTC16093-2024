package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class AutoBlueNear extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        int selected_line=1;
        int auto_mode=1;//1:2+0 1:2+1 2:2+2 2:2+3
        int auto_route=1;//1:center 2:edge
        int park=1;//1:outer 2:inner 3:no parking
        startSide = PROXIMAL;
        side_color = BLUE;
        drop_side = RIGHT;
        boolean y_pressed=false;
        boolean a_pressed=false;
        boolean xb_pressed=false;
        while(!opModeIsActive()){
            telemetry.addLine("------------------------------------------------");
            telemetry.addLine("|"+(selected_line==1?">":"  ")+"自动跑几趟:"+(startSide==PROXIMAL?(auto_mode==1?"2+0":"2+2"):(auto_mode==1?"2+1":"2+3"))+"|");
            telemetry.addLine("|"+(selected_line==2?">":"  ")+"自动路径:"+(auto_route==1?"中间跑":"边上跑")+"|");
            telemetry.addLine("|"+(selected_line==3?">":"  ")+"放那边:"+(drop_side==LEFT?"左":"右")+"|");
            telemetry.addLine("|"+(selected_line==4?">":"  ")+"停靠:"+(park==3?"停板前":(park==1?"板外侧":"板内侧"))+"|");
            telemetry.addLine("------------------------------------------------");
            telemetry.update();
            if(gamepad1.y&&!y_pressed){
                selected_line=Math.max(1,selected_line-1);
                y_pressed=true;
            }
            if(gamepad1.a&&!a_pressed){
                selected_line=Math.min(4,selected_line+1);
                a_pressed=true;
            }
            if((gamepad1.b||gamepad1.x)&&!xb_pressed){
                auto_mode=selected_line==1?(auto_mode==1?2:1):auto_mode;
                auto_route=selected_line==2?(auto_route==1?2:1):auto_route;
                drop_side=selected_line==3?(drop_side==LEFT?RIGHT:LEFT):drop_side;
                park=selected_line==4?(gamepad1.b?Math.max(1,park-1):Math.min(3,park+1)):park;
                xb_pressed=true;
            }
            y_pressed=(y_pressed?gamepad1.y:y_pressed);
            a_pressed=(a_pressed?gamepad1.a:a_pressed);
            xb_pressed=(xb_pressed?gamepad1.x||gamepad1.b:xb_pressed);
        }
        if(auto_mode==1){
            if(auto_route==1){
                if(drop_side==LEFT){//2+0 走中间 放左边
                    BackDrop_blueRight_y = 22.2;
                    BackDrop_blueCenter_y = 28.3;
                    BackDrop_blueLeft_y = 33.9;
                    startSide = PROXIMAL;
                    side_color = BLUE;
                    initHardware();
                    spikeMarkDump();
                    putOnSpikeMark();
                    backDropDump();
                    upper.setArmPosition(2100);
                    sleep(500);
                    upper.grab2_open();
                    sleep(300);
                    setUpAuto();
                    //extraCredit();
                    ecByCenter_farCenter();
                    //ec_lowFar_edgeSpline_blue()
                    raiseArm_slow(armPosUpward);
                    putOnBackDrop_grab2();
                    raiseArm_slow(armPosUpward-30);
                    sleep(500);
                    upper.setArmPosition_slow(armPosUpward-120);
                    setUpAuto();
                    backTo_StartPos();
                    sleep(1000);
                    parking(park);
                }else if(drop_side==RIGHT){//2+0 走中间 放右边
                    startSide = PROXIMAL;
                    side_color = BLUE;
                    drop_side = RIGHT;
                    initHardware();
                    spikeMarkDump();
                    putOnSpikeMark();

                    backDropDump();

                    upper.setArmPosition(2100);
                    sleep(500);
                    upper.grab2_open();
                    sleep(300);
                    setUpAuto();

                    //extraCredit();
                    ecByCenter_farCenter();
                    //ec_lowFar_edgeSpline_blue();
                    raiseArm_slow(armPosUpward);
                    putOnBackDrop_grab2();
                    raiseArm_slow(armPosUpward-30);
                    sleep(500);
                    upper.setArmPosition_slow(armPosUpward-120);
                    setUpAuto();
                    backTo_StartPos();
                    sleep(1000);
                    parking(park);
                }
            }else if(auto_route==2){
                if(drop_side==LEFT){//2+0 走边上 放左边

                }else if(drop_side==RIGHT){//2+0 走边上 放右边

                }
            }
        }else if(auto_mode==2){
            if(auto_route==1){
                if(drop_side==LEFT){//2+2 走中间 放左边

                }else if(drop_side==RIGHT){//2+2 走中间 放右边

                }
            }else if(auto_route==2){
                if(drop_side==LEFT){//2+2 走边上 放左边

                }else if(drop_side==RIGHT){//2+2 走边上 放右边

                }
            }
        }
    }

}

