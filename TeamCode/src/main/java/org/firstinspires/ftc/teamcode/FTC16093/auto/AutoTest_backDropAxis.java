package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest_backDropAxis extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = DISTAL;
        side_color = BLUE;
        drop_side = LEFT;

        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();
        intakeDistal();
        sleep(wait_time);
        distal_backDropAxis();
        sleep(300);
        upper.setArmPosition(2040);
        sleep(1000);
        upper.putOnBackDrop();
        upper.setArmPosition(1930);
        backDrop_move1Pixel();
        upper.release_extra();
        sleep(200);
        setUpAuto();
        sleep(1500);
        parking(3);
    }
}
