package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous (group = "AutoBlueFar")
@Disabled
public class AutoBlueFar_LeftDrop_sideIntake_sideBack extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{

        startSide = DISTAL;
        side_color = BLUE;
        drop_side = LEFT;
        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();

        //DistalBackDropDump();
        distal_intakeSide_sideBack();
        upper.setArmPosition(2050);
        sleep(1000);
        upper.putOnBackDrop();

        upper.setArmPosition(2000);
        backDrop_move();
        upper.release_extra();
        sleep(200);
        setUpAuto();
        sleep(1500);
    }
}
