package org.firstinspires.ftc.teamcode.FTC16093.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest_backDropAxis_near extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = DISTAL;
        side_color = BLUE;
        drop_side = RIGHT;

        initHardware();
        moveToSpikeMark();
        upper.putOnSpikeMark();

        delay(500);
        upper.putOnBackDrop();
        upper.setArmPosition(0);
        delay(500);
        //extraCredit();
        intakeGate();
        //ec_lowFar_edgeSpline_blue();
        upper.drop_upward();

        upper.setArmPosition(500);
        upper.wrist_origin();

        backDrop_move1Pixel();
        upper.release_extra();
        delay(200);
        setUpAuto();
        delay(1500);
        parking(3);
    }
}
