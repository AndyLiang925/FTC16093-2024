package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoRedNear")
@Config
public class AutoRedNear_RightDrop_center extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = RED;
        drop_side = RIGHT;
        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();
        moveToBackDrop();
        delay(500);
        upper.putOnBackDrop();
        upper.setArmPosition(0);
        delay(500);
        //extraCredit();
        intakeGate();
        //ec_lowFar_edgeSpline_blue();
        upper.drop_upward();

        upper.setArmPosition(500);
        upper.wrist_to_middle();


        delay(1000);
    }
}