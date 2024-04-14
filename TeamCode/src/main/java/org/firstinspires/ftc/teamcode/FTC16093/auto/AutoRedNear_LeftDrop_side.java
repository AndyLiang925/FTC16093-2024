package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedNear")
@Config
public class AutoRedNear_LeftDrop_side extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;
        drop_side = LEFT;
        initHardware();

        spikeMarkDump();
        putOnSpikeMark();
        backDropDump();

        upper.setArmPosition(2146);
        sleep(500);
        upper.grab2_open();
        sleep(300);
        setUpAuto();

        //backTo_StartPos();
    }
}
