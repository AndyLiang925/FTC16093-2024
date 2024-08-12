package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedNear")
@Config
public class AutoRedNear_side extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;

        // 初始化
        initHardware();

        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();


        moveToBackDrop();
        sleep(500);
        upper.putOnBackDrop();
        sleep(300);
        setUpAuto();
        ec_lowFar_edgeSpline_blue();
    }
}
