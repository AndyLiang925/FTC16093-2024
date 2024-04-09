package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;

@Autonomous
public class DropTest16093 extends AutoMaster {

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        //prepare_drop_upward();
        //raiseArm(1800);
        putOnBackDrop_grab2();
        sleep(300);
        setUpAuto();
        sleep(3000);
    }

}
