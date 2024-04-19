package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest_backDropAxis extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = DISTAL;
        side_color = RED;
        drop_side = RIGHT;

        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();
        intakeDistal();
        sleep(wait_time);
        distal_backDropAxis();
        sleep(300);
        upper.putOnBackDrop();
        upper.setArmPosition(4100);
        backDrop_move1Pixel();
        upper.release_extra();
        sleep(200);
        setUpAuto();
        sleep(1500);
        parking(3);
    }
}
