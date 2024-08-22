package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoBlueNear")
@Config
public class AutoBlueNear_side extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = BLUE;

        initHardware();

        moveToSpikeMark();
        upper.putOnSpikeMark();

        moveToDropYellow_Near();

        intakeSide();
        upper.drop_upward();

        parking(3);
    }
}
