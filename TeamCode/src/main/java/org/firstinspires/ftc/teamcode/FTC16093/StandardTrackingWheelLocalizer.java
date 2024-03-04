//package org.firstinspires.ftc.teamcode.FTC16093;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.util.Encoder;
//
//import java.util.Arrays;
//import java.util.List;
//
///*
// * Sample tracking wheel localizer implementation assuming the standard configuration:
// *
// *    /--------------\
// *    |     ____     |
// *    |     ----     |
// *    | ||        || |
// *    | ||        || |
// *    |              |
// *    |              |
// *    \--------------/
// *
// */
//@Config
//@Disabled
//public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
//    public static double TICKS_PER_REV = 2000; //2000
//    public static double WHEEL_RADIUS = 0.945; // in
//    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
//
//    public static double LATERAL_DISTANCE = 6.7; // in; distance between the left and right wheels
//    public static double FORWARD_OFFSET = -3.9375; // in; offset of the lateral wheel
//
//    private Encoder leftEncoder, rightEncoder, frontEncoder;
//
//    public static double X_MULTIPLIER = 0.9874489832255648;// Multiplier in the X direction
//    public static double Y_MULTIPLIER = 1; //1.0002493014638826818108086960139 // Multiplier in the Y direction
//
//    private List<Integer> lastEncPositions, lastEncVels;
//
//    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
//        super(Arrays.asList(
//                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
//                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
//                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
//        ));
//
//        lastEncPositions = lastTrackingEncPositions;
//        lastEncVels = lastTrackingEncVels;
//
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rearLeft"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        rightEncoder.setDirection(Encoder.Direction.FORWARD);
//        frontEncoder.setDirection(Encoder.Direction.FORWARD);
//        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
//    }
//
//    public static double encoderTicksToInches(double ticks) {
//        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
//    }
//
//    @NonNull
//    @Override
//    public List<Double> getWheelPositions() {
//        int leftPos = leftEncoder.getCurrentPosition();
//        int rightPos = rightEncoder.getCurrentPosition();
//        int frontPos = frontEncoder.getCurrentPosition();
//
//        lastEncPositions.clear();
//        lastEncPositions.add(leftPos);
//        lastEncPositions.add(rightPos);
//        lastEncPositions.add(frontPos);
//
//        return Arrays.asList(
//                encoderTicksToInches(leftPos) * X_MULTIPLIER,
//                encoderTicksToInches(rightPos) * X_MULTIPLIER,
//                encoderTicksToInches(frontPos) * Y_MULTIPLIER
//        );
//    }
//
//    @NonNull
//    @Override
//    public List<Double> getWheelVelocities() {
//        int leftVel = (int) leftEncoder.getCorrectedVelocity();
//        int rightVel = (int) rightEncoder.getCorrectedVelocity();
//        int frontVel = (int) frontEncoder.getCorrectedVelocity();
//
//        lastEncVels.clear();
//        lastEncVels.add(leftVel);
//        lastEncVels.add(rightVel);
//        lastEncVels.add(frontVel);
//
//        return Arrays.asList(
//                encoderTicksToInches(leftVel) * X_MULTIPLIER,
//                encoderTicksToInches(rightVel) * X_MULTIPLIER,
//                encoderTicksToInches(frontVel) * Y_MULTIPLIER
//        );
//    }
//}
