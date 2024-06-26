package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous (group = "AutoRedFar")
@Disabled
public class AutoRedFar_RightDrop_centerIntake_sideBack extends AutoMaster {
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
        distal_edgeBack();
        sleep(300);
        upper.setArmPosition(2077);
        sleep(1000);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        sleep(300);

        backDrop_move();

        upper.putOnBackDrop();
        sleep(200);
        setUpAuto();
        sleep(1500);
        //parking(2);
        //ec_far_putOnGround();

    }
}

