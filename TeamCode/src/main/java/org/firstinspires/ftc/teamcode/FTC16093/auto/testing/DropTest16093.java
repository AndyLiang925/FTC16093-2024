//package org.firstinspires.ftc.teamcode.FTC16093.auto.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.FTC16093.auto.AutoMaster;
//
//@Autonomous
//public class DropTest16093 extends AutoMaster {
//
//    @Override
//    public void runOpMode() throws InterruptedException{
//        initHardware();
//        //prepare_drop_upward();
//        //raiseArm(1800);
//        upper.wrist_to_upward();
//        upper.setArmPosition(1500);
//        sleep(1000);
//
//        raiseArm_slow(armPosUpward);
//        sleep(1000);
//        putOnBackDrop_grab1();
//        sleep(300);
//        upper.wrist_to_upward_drop();
//        sleep(200);
//        raiseArm_slow(armPosUpward-armPos_delta);
//        sleep(500);
//        upper.setArmPosition_slow(armPosUpward-120);
//
//        setUpAuto();
//        sleep(3000);
//    }
//
//}
