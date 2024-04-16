package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoBlueNear")
@Config
public class AutoBlueNear_RightDrop_center extends AutoMaster {
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
        sleep(1000);
        upper.grab2_open();
        sleep(300);
        setUpAuto();

        ecByCenter_farCenter();
        drop_upward_grab2();
        setUpAuto();
        sleep(1000);
        parking(1);
    }
}
