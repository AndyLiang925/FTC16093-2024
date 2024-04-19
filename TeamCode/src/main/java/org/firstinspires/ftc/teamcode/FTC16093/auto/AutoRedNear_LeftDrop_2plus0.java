package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedNear")
@Config
public class AutoRedNear_LeftDrop_2plus0 extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;
        drop_side = LEFT;
        initHardware();

        moveToSpikeMark();
        upper.putOnSpikeMark();
        moveToBackDrop();

//        upper.setArmPosition_slow(4200);
        sleep(1000);
        upper.putOnBackDrop();
        setUpAuto();
        parking(1);
        //backTo_StartPos();
    }
}

