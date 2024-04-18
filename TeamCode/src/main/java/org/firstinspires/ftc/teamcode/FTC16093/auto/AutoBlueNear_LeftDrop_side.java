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
        upper.putOnSpikeMark();
        backDropDump();

        upper.setArmPosition_slow(2077);
        sleep(500);
        upper.putOnBackDrop();
        setUpAuto();

        ec_lowFar_edgeSpline_blue();
        upper.drop_upward();
        setUpAuto();
        sleep(1500);
        parking(3);
    }
}
