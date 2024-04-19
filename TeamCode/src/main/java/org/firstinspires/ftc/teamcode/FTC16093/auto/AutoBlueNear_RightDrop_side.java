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
        moveToSpikeMark();
        upper.putOnSpikeMark();
        moveToBackDrop();

        upper.setArmPosition_slow(2077);
        sleep(1000);
        upper.putOnBackDrop();
        setUpAuto();
        ec_lowFar_edgeSpline_blue();
        upper.drop_upward();
        setUpAuto();
        sleep(1000);
        parking(3);
    }
}