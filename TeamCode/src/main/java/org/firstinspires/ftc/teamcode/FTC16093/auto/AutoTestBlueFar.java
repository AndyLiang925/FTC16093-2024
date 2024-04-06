package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestBlueFar extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;

        initHardware();
        //upper.init_armExpand();

        spikeMarkDump();
        putOnSpikeMark();
        DistalBackDropDump();
        sleep(300);
        upper.setArmPosition(2077);
        sleep(wait_time);

        upper.grab2_open(); //yellow
        upper.setArmPosition(1980);
        sleep(200);

        backDrop_move();

        putOnBackDrop_grab1();
        sleep(sleep_3);
        setUpAuto();
        sleep(1500);
        //parking(2);
        ecByCenter();

        upper.setArmPosition(1980);
        sleep(800);
        putOnBackDrop_grab1();
        sleep(sleep_3);
        setUpAuto();
        sleep(1500);
    }
}

