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
import org.opencv.core.Mat;

import java.util.Objects;
@Config
public class VerticalSlides {
    public static double kP = 0.006;
    public static  double kI = 0;
    public static double kD = 0;
    public static  double kF = 0;
    public static double POSITION_TOLERANCE = 20;
    public enum SlidePositions {
        DOWN,
        BUCKET,
        CHAMBER
    }
    double targetPos;
    PIDFController controller = new PIDFController(kP, kI, kD, kF);

    SlidePositions slidePosition = SlidePositions.DOWN;
    DcMotor rightSlide;
    DcMotor leftSlide;
    public static int MAX = 2250;
    int down = 0;
    public static int CHAMBER = 1650;
    double maxPower = 1;
    double lockPower = .1;
    public static double downPower = .3;
    boolean lock = false;
    int lockPosition;

    public void initiate(HardwareMap hardwareMap) {
        rightSlide = hardwareMap.dcMotor.get("RightSlide");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide = hardwareMap.dcMotor.get("LeftSlide");
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller.setTolerance(POSITION_TOLERANCE);
    }

    public void update() {
        controller.setP(kP);
        controller.setD(kD);
        controller.setI(kI);
        controller.setF(kF);
        switch (slidePosition) {
            case DOWN:
                targetPos = down;
                maxPower =downPower;
                break;
            case BUCKET:
                targetPos = MAX;
                maxPower = 1;
                break;
            case CHAMBER:
                targetPos = CHAMBER;
                maxPower =1;
                break;
        }
        double power = controller.calculate(leftSlide.getCurrentPosition(),targetPos);
        if (Math.abs(power) > maxPower){
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
            leftSlide.setTargetPosition(MAX);
            rightSlide.setTargetPosition(-MAX);
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
