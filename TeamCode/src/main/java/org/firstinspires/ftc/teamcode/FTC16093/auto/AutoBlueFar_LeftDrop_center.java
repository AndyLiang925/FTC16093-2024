package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.drive.BarkMecanumDrive;

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
        upper.setArmPosition(2060); //2077
        sleep(1000);
        upper.putOnBackDrop();
        upper.setArmPosition(2030);
        backDrop_move();
        upper.release_extra();
        sleep(200);
        setUpAuto();
        sleep(1500);
        parking(2);
        //ecByCenter_far();
        //ec_far_putOnGround();
    }
}

