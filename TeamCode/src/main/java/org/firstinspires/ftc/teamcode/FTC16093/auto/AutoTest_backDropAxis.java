package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest_backDropAxis extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = DISTAL;
        side_color = RED;
        initHardware();

        // 放紫片
        moveToSpikeMark();
        upper.putOnSpikeMark();
        delay(wait_time);

        // 夹1个白片
        intakeDistal();

        distal_backDropAxis();
        delay(1500);
        parking(3);
    }
}
