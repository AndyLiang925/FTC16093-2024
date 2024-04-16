package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoBlueNear")
@Config
public class AutoBlueNear_RightDrop_side extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = BLUE;
        drop_side = RIGHT;
        initHardware();
        spikeMarkDump();
        putOnSpikeMark();
        backDropDump();

        upper.setArmPosition_slow(2077);
        sleep(800);
        upper.grab2_open();
        sleep(300);
        setUpAuto();

        ec_lowFar_edgeSpline_blue();
        raiseArm_slow(armPosUpward);
        putOnBackDrop_grab2();
        raiseArm_slow(armPosUpward-30);
        sleep(500);
        upper.setArmPosition_slow(armPosUpward-120);
        setUpAuto();
        sleep(1000);
        parking(1);
    }
}