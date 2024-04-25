package org.firstinspires.ftc.teamcode.FTC16093.uppersystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import XCYOS.Component;
import XCYOS.Task;
import XCYOS.TaskChainBuilder;

@Config
public class NewSuperStructure implements Component {
    public enum Grabber {
        RIGHT,
        LEFT,
        ALL,
        PURPLE,
        YELLOW
    }

    private boolean sideIsRed;
    private DcMotorEx armMotor;
    private DcMotorEx slideMotor;
    private DcMotorEx armExternalEnc;
    private Servo grabber1, grabber2, wrist;
    public static double grab1_close = 0.85, grab2_close = 0.48, grabber_open = -0.26, grabber_release = -0.15;

    public static final int MAX_SLIDE_LENGTH = 573;
    public static final int MIN_SLIDE_LENGTH = -6;
    public static int ARM_INTAKE_POS = 470;
    public static final int MIM_ARM_BACKDROP = 4000;
    public static final double MIN_GRABBER_DISTANCE = 4;
    public static final double MAX_GRABBER_DISTANCE = 0;

    public NewSuperStructure() {
        armPidCtrl = new PIDFController(armPidConf);
    }

    public void setSideIsRed(boolean sideIsRed) {
        this.sideIsRed = sideIsRed;
    }

    @Override
    public void setUp(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        slideMotor = hardwareMap.get(DcMotorEx.class, "armExpand");
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber1 = hardwareMap.get(Servo.class, "grab1");
        grabber2 = hardwareMap.get(Servo.class, "grab2");
        armExternalEnc = hardwareMap.get(DcMotorEx.class, "hangRight");

        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Task toOriginal() {
        return new TaskChainBuilder(2, this)
                .add(() -> {
                    setArmPosition(0);
                    setSlide(MIN_SLIDE_LENGTH, 1);
                    setWristPosition(wrt_middle_pos);
                    closeGrabber(Grabber.ALL);
                })
                .end().getBase();
    }

    public Task resetSliderEnc() {
        return new TaskChainBuilder(1, this)
                .setComponent()
                .add(() -> {
                    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slideMotor.setPower(-0.3);
                })
                .sleep(200)
                .add(() -> {
                    slideMotor.setPower(0);
                    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setSlide(MIN_SLIDE_LENGTH, 1);
                })
                .end().getBase();
    }

    public Task resetArmEnc() {
        TaskChainBuilder tb = new TaskChainBuilder(1, this);
        return tb
                .setPriority(-1)
                .add(() -> armMotor.setPower(-0.2))
                .sleep(500)
                .add(() -> {
                    armMotor.setPower(0);
                    armExternalEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armExternalEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                })
                .end().getBase();
    }

    public Task toIntakeGround() {
        TaskChainBuilder tb = new TaskChainBuilder(1, this);
        return tb
                .add(() -> setArmPosition(ARM_INTAKE_POS))
                .sleep(200)
                .add(() -> {
                    setSlide(MAX_SLIDE_LENGTH, 1);
                    setWristPosition(wrt_intake_max_pos);
                    openGrabber(Grabber.ALL);
                })
                .sleep(300)
                .end()
                .getBase();
    }

    public Task intakeBack() {
        return new TaskChainBuilder(1, this)
                .add(() -> {
                    setArmPosition(ARM_INTAKE_POS + 100);
                    setWristPosition(wrt_middle_pos);
                    setSlide(0, 1);
                })
                .sleep(300)
                .add(() -> setArmPosition(0))
                .end().getBase();
    }

    public Task toPlaceMarker() {
        TaskChainBuilder tb = new TaskChainBuilder(1, this);
        return tb
                .add(() -> {
                    setArmPosition(ARM_INTAKE_POS);
                    setWristPosition(wrt_intake_min_pos);
                })
                .sleep(300)
                .add(() -> setSlide(MAX_SLIDE_LENGTH, 1))
                .waitFor(() -> slideMotor.getCurrentPosition() > (MAX_SLIDE_LENGTH - 100))
                .end()
                .getBase();
    }

    public Task release(Grabber grabber) {
        TaskChainBuilder tb = new TaskChainBuilder(1, this);
        return tb
                .add(() -> openGrabber(grabber))
                .sleep(300)
                .end().getBase();
    }

    public Task toBackDrop(int height) {
        return new TaskChainBuilder(1, this)
                .add(() -> setArmPosition(MIM_ARM_BACKDROP))
                .sleep(500)
                .add(() -> {
                    setSlide(height, 1);
                    setWristPosition(wrt_drop_high_pos);
                })
                .end()
                .getBase();
    }

    public void openGrabber(Grabber grabber) {
        setGrabber(grabber, grabber_open);
    }

    public void closeGrabber(Grabber grabber) {
        setGrabber(grabber, 0);
    }

    public void releaseGrabber(Grabber grabber) {
        setGrabber(grabber, grabber_release);
    }

    private void setGrabber(Grabber grabber, double diff) {
        switch (grabber) {
            case ALL:
                grabber1.setPosition(grab1_close + diff);
                grabber2.setPosition(grab2_close + diff);
                return;
            case LEFT:
                grabber2.setPosition(grab2_close + diff);
                return;
            case RIGHT:
                grabber1.setPosition(grab1_close + diff);
                return;
            case PURPLE:
                if (sideIsRed) {
                    grabber2.setPosition(grab2_close + diff);
                } else {
                    grabber1.setPosition(grab1_close + diff);
                }
                return;
            case YELLOW:
                if (sideIsRed) {
                    grabber1.setPosition(grab1_close + diff);
                } else {
                    grabber2.setPosition(grab2_close + diff);
                }
        }
    }

    public boolean grabberIsClosed(Grabber grabber) {
        switch (grabber) {
            case LEFT:
                return Math.abs(grabber2.getPosition() - grab2_close) < 0.001;
            case RIGHT:
                return Math.abs(grabber1.getPosition() - grab1_close) < 0.001;
            default:
                return false;
        }
    }

    public void setGroundIntakeLength(double distance){
        distance=Range.clip(distance,MIN_GRABBER_DISTANCE,MAX_GRABBER_DISTANCE);
        double angle = Math.atan2(8.268,distance-2.9134);
        setArmPosition((int) (23.03*angle));
        setSlide((int) (39.65 *(distance-2.9134) / Math.cos(angle)),0.85);
    }

    public void setSlide(int length, double power) {
        slideMotor.setTargetPosition(length);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(power);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    public void setArmPosition(int pos) {
        armTargetPosition = pos;
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (armExternalEnc.getCurrentPosition() <= 800 && pos <= armExternalEnc.getCurrentPosition()) {
            armPidCtrl.setOutputBounds(-0.3, 0.3);
        } else if (armExternalEnc.getCurrentPosition() < 3000 && pos >= armExternalEnc.getCurrentPosition()) {
            armPidCtrl.setOutputBounds(-0.8, 0.8);
        } else if (pos >= armExternalEnc.getCurrentPosition()) {
            armPidCtrl.setOutputBounds(-0.5, 0.5);
        } else {
            armPidCtrl.setOutputBounds(-0.8, 0.8);
        }
    }

    @Override
    public void update() {
        armMotor.setPower(armPidCtrl.update(armExternalEnc.getCurrentPosition() - armTargetPosition));
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double armTargetPosition;
    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController armPidCtrl;
    public static final double wrt_intake_min_pos = 0.51;
    public static double wrt_intake_max_pos = .45;
    public static final double wrt_middle_pos = 0.86;
    public static final double wrt_drop_low_pos = 0.9;
    public static double wrt_drop_high_pos = 0.27;
}
