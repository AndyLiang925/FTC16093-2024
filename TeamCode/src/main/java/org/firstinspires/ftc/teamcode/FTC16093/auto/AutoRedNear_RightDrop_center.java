package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoRedNear")
@Config
public class AutoRedNear_RightDrop_center extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;
        drop_side = RIGHT;
        initHardware();
        spikeMarkDump();
        upper.putOnSpikeMark();
        backDropDump();

        upper.setArmPosition(2077);
        sleep(500);
        upper.grab2_open();
        sleep(300);
        setUpAuto();

        //extraCredit();
        ecByCenter_farCenter();
        //ec_lowFar_edgeSpline_blue();
        upper.putOnBackDrop();

        setUpAuto();
        sleep(1000);
        //parking(1);
        //backTo_StartPos();
    }
}