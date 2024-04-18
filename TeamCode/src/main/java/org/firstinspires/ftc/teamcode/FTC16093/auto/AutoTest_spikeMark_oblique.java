package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest_spikeMark_oblique extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = DISTAL;
        side_color = RED;
        drop_side = LEFT;

        initHardware();
        spikeMarkDump();
        upper.putOnSpikeMark();
    }
}
