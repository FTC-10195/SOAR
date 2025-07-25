package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class VerticalSlides {
    public static double kP = 0.006;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static double POSITION_TOLERANCE = 20;

    public enum SlidePositions {
        DOWN,
        BUCKET,
        CHAMBER,
        BARNACLE
    }

    double targetPos;
    public boolean secondaryBucket = false;
    PIDFController pidfController = new PIDFController(kP, kI, kD, kF);

    SlidePositions slidePosition = SlidePositions.DOWN;
    DcMotor rightSlide;
    DcMotor leftSlide;
    public static int BUCKET = 1950;
    public static int BUCKET_SECONDARY = 1700;
    int down = 0;
    public static int CHAMBER = 1600;
    public static int BARNACLE = 800;
    double maxPower = 1;
    double lockPower = .1;
    public int offset = 0;
    public int offsetGain = 20;
    public static double downPower = .4;
    boolean lock = false;
    int lockPosition;

    public void initiate(HardwareMap hardwareMap) {
        rightSlide = hardwareMap.dcMotor.get("RightSlide");
        leftSlide = hardwareMap.dcMotor.get("LeftSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pidfController.setTolerance(POSITION_TOLERANCE);
    }

    public void update() {
        pidfController.setP(kP);
        pidfController.setD(kD);
        pidfController.setI(kI);
        pidfController.setF(kF);
        switch (slidePosition) {
            case DOWN:
                targetPos = down + offset;
                maxPower = downPower;
                break;
            case BUCKET:
                targetPos = BUCKET + offset;
                if (secondaryBucket){
                    targetPos = BUCKET_SECONDARY + offset;
                }
                maxPower = 1;
                break;
            case CHAMBER:
                targetPos = CHAMBER + offset;
                maxPower = 1;
                break;
            case BARNACLE:
                targetPos = BARNACLE + offset;
                maxPower = 1;
                break;
        }
        double power = pidfController.calculate(leftSlide.getCurrentPosition(), targetPos);
        if (Math.abs(power) > maxPower) {
            power = maxPower * Math.signum(power);
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void setSlidePosition(SlidePositions slidePosition) {
        this.slidePosition = slidePosition;
    }

    public double error() {
        return Math.abs(leftSlide.getTargetPosition() - leftSlide.getCurrentPosition());
    }

    public void manual(double power, boolean overide, Telemetry telemetry) {
        if (power > 0.1) {
            leftSlide.setTargetPosition(BUCKET);
            rightSlide.setTargetPosition(-BUCKET);
            lock = false;
        } else if (power < -.1) {
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            lock = false;
        } else {
            if (!lock) {
                lock = true;
                lockPosition = leftSlide.getCurrentPosition();
            }
            leftSlide.setTargetPosition(lockPosition);
            rightSlide.setTargetPosition(-lockPosition);
            if (!overide) {
                power = lockPower;
            }
        }
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        telemetry.addData("leftslidepos", leftSlide.getCurrentPosition());
        telemetry.addData("rightslidepos", rightSlide.getCurrentPosition());
        telemetry.addData("targetPos", leftSlide.getTargetPosition());
        telemetry.addData("LockPos", lockPosition);
        telemetry.addData("Locked", lock);
        telemetry.addData("Power:", power);
    }

    public void reset(boolean options) {
        if (options) {
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void status(Telemetry telemetry) {
        telemetry.addData("LeftSlidePos", leftSlide.getCurrentPosition());
        telemetry.addData("RightSlidePos", rightSlide.getCurrentPosition());
    }

    public Action updateAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return true;
            }
        };
    }

    public Action slideAction(SlidePositions state) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setSlidePosition(state);
                return false;
            }
        };
    }


}
