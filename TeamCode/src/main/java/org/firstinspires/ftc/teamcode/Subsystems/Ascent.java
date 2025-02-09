package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

import java.util.Objects;
@Config
public class Ascent {
    public static double kP = 0.006;
    public static  double kI = 0;
    public static double kD = 0;
    public static  double kF = 0;
    public static double POSITION_TOLERANCE = 20;
    public enum ClimbPositions {
        DOWN,
        PLACE,
        MAX
    }
    double targetPos;
    PIDFController controller = new PIDFController(kP, kI, kD, kF);

    public ClimbPositions climbPosition = ClimbPositions.DOWN;
    DcMotor rightClimb;
    DcMotor leftClimb;
    Servo rightServo;
    Servo leftServo;
    public static double BACK = 0.3;
    public static double FORWARD = 0.9;
    public static int MAX = 2250;
    public static int DOWN = 0;
    double maxPower = 1;
    double lockPower = .8;
    public static double downPower = 0;
    boolean lock = false;
    int lockPosition;

    public void initiate(HardwareMap hardwareMap) {
        rightClimb = hardwareMap.dcMotor.get("RightClimb");
        rightClimb.setDirection(DcMotorSimple.Direction.REVERSE);
        leftClimb = hardwareMap.dcMotor.get("LeftClimb");
        rightClimb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftClimb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightServo = hardwareMap.servo.get("RightHook");
        leftServo = hardwareMap.servo.get("LeftHook");
        controller.setTolerance(POSITION_TOLERANCE);
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("leftClimb",leftClimb.getCurrentPosition());
        telemetry.addData("rightClimb",rightClimb.getCurrentPosition());
        controller.setP(kP);
        controller.setD(kD);
        controller.setI(kI);
        controller.setF(kF);
        switch (climbPosition) {
            case DOWN:
                targetPos = DOWN;
                maxPower =downPower;
                rightServo.setPosition(BACK);
                break;
            case PLACE:
                targetPos = DOWN;
                maxPower =downPower;
                rightServo.setPosition(FORWARD);
                break;
            case MAX:
                targetPos = MAX;
                rightServo.setPosition(BACK);
                maxPower = 1;
                break;
        }
        double power = controller.calculate(leftClimb.getCurrentPosition(),targetPos);
        if (Math.abs(power) > maxPower){
            power = maxPower * Math.signum(power);
        }
        leftServo.setPosition(1-rightServo.getPosition());
        rightClimb.setPower(power);
        leftClimb.setPower(power);
    }

    public void setClimb(ClimbPositions climbPosition) {
        this.climbPosition = climbPosition;
    }
    public void reset(boolean options) {
        if (options) {
            leftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
