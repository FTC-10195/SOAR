package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class DriveTrain {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    public static double frP=1;
    public static double flP = 1;
    public static double brP = 1;
    public static double blP = 1;

    public void initiate(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("motor0");
        frontRightMotor = hardwareMap.dcMotor.get("motor1");
        backLeftMotor = hardwareMap.dcMotor.get("motor2");
        backRightMotor = hardwareMap.dcMotor.get("motor3");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double x, double y, double rx, Telemetry telemetry) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator)*flP;
        double backLeftPower = ((y - x + rx) / denominator)*blP;
        double frontRightPower = ((y - x - rx) / denominator)*frP;
        double backRightPower = ((y + x - rx) / denominator)*brP;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

    }
    public void runRight(double power) {
        frontLeftMotor.setPower(power*flP);
        backLeftMotor.setPower(-power*blP);
        frontRightMotor.setPower(-power*frP);
        backRightMotor.setPower(power*brP);

    }
}
