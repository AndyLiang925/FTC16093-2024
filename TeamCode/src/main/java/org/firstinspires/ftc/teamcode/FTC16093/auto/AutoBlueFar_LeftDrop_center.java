package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoBlueFar")
public class AutoBlueFar_LeftDrop_center extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;
        drop_side = LEFT;
        initHardware();
        spikeMarkDump();
        upper.putOnSpikeMark();

        //DistalBackDropDump();
        distal_intake_center();
        sleep(wait_time);
        distal_backDropDump_center();

        //DistalBackDropDump_farGrab();
        sleep(300);
        upper.setArmPosition(2050); //2077
        sleep(1000);
        upper.putOnBackDrop();
        upper.setArmPosition(2000);
        backDrop_move();
        upper.release_extra();
        sleep(200);
        setUpAuto();
        sleep(1500);
        parking(3);
        //ecByCenter_far();
        //ec_far_putOnGround();
    }
}

