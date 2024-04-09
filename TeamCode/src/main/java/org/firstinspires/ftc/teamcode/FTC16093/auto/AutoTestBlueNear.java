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

        upper.setArmPosition(2050);
        sleep(500);
        upper.grab2_open();
        sleep(300);
        setUpAuto();

        //extraCredit();
        ecByCenter_farCenter();
        //ec_lowFar_edgeSpline_blue();
        raiseArm(armPosUpward);
        putOnBackDrop_grab2();
        sleep(300);
        setUpAuto();
        sleep(1000);
        parking(1);
    }

}

