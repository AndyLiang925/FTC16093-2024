package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoBlueNear")
@Config
public class AutoBlueNear_LeftDrop_side extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = BLUE;
        drop_side = LEFT;
        initHardware();
        spikeMarkDump();
        putOnSpikeMark();
        backDropDump();

        upper.setArmPosition_slow(2077);
        sleep(500);
        upper.grab2_open();
        sleep(300);
        setUpAuto();

        ec_lowFar_edgeSpline_blue();
        drop_upward_grab1();
        setUpAuto();
        sleep(1500);
        parking(3);
    }
}
