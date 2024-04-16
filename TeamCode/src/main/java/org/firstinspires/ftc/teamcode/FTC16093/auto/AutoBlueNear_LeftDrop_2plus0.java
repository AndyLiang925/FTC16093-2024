package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoBlueNear")
@Config
public class AutoBlueNear_LeftDrop_2plus0 extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = BLUE;
        drop_side = LEFT;
        initHardware();

        spikeMarkDump();
        putOnSpikeMark();

        backDropDump();

        upper.setArmPosition(2077);
        sleep(1000);
        upper.grab2_open();
        sleep(300);
        setUpAuto();
        backTo_StartPos();
    }

}
