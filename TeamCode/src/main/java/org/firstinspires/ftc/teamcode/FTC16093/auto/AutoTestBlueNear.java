package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class AutoTestBlueNear extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

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
        //ec_lowFar_edgeSpline_blue();
        raiseArm_slow(armPosUpward);
        putOnBackDrop_grab2();
        raiseArm_slow(armPosUpward-30);
        sleep(500);
        upper.setArmPosition_slow(armPosUpward-120);
        setUpAuto();
        backTo_StartPos();
//        sleep(1000);
//        parking(1);
    }

}

