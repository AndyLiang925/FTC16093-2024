package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoBlueFar")
public class AutoBlueFar_LeftDrop_side extends AutoMaster {
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
        distal_edgeBack();

        sleep(1000);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        backDrop_move();
        upper.putOnBackDrop();
        sleep(200);
        setUpAuto();
        sleep(1500);
    }
}
