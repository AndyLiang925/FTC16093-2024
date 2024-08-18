package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoBlueNear")
@Config
public class AutoBlueNear_2plus0 extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = BLUE;
        drop_side = LEFT;
        initHardware();

        moveToSpikeMark();
        upper.putOnSpikeMark();
        moveToDropYellow_Near();

        upper.setArmPosition(2077);
        sleep(1000);
        upper.dropYellow();
        init_setUpAuto();
        sleep(1);
    }

}
