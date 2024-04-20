package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "AutoBlueNear")
@Config
public class AutoBlueNear_LeftDrop_center extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = PROXIMAL;
        side_color = BLUE;
        drop_side = LEFT;
        initHardware();

        moveToSpikeMark();
        upper.putOnSpikeMark();
        moveToBackDrop();

        upper.putOnBackDrop();
        upper.setArmPosition(0);
        intakeGate();
        upper.drop_upward();
        upper.setArmPosition(0);
        delay(1000);
        parking(3);
    }
}
