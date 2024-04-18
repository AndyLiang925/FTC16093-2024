package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "AutoRedFar")
public class AutoRedFar_LeftDrop_centerIntake_gate extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = RED;
        drop_side = LEFT;
        initHardware();

        spikeMarkDump();
        upper.putOnSpikeMark();

        distal_intake_center();
        sleep(wait_time);
        distal_backDropDump_center();
        sleep(300);
        upper.setArmPosition(2077);
        sleep(1000);
        upper.grab2_open(); //yellow
        upper.setArmPosition(2000);
        sleep(300);

        backDrop_move();

        upper.drop_upward();
        sleep(200);
        setUpAuto();
        sleep(1500);
        //parking(2);
        //ec_far_putOnGround();

    }
}

