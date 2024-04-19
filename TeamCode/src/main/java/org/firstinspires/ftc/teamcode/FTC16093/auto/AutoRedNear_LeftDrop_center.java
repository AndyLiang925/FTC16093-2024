package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoRedNear")
@Config
public class AutoRedNear_LeftDrop_center extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;
        drop_side = LEFT;
        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();
        moveToBackDrop();
        sleep(500);
        upper.putOnBackDrop();
        upper.setArmPosition_slow(0);
        delay(500);
        //extraCredit();
        intakeGate();
        //ec_lowFar_edgeSpline_blue();
        upper.drop_upward();
        upper.setArmPosition_slow(0);
        sleep(1000);
        //parking(1);
        //backTo_StartPos();
    }
}
