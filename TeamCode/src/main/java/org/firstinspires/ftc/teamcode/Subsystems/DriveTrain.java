package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class DriveTrain {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    double x, y;
    public void initiate(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("motor0");
        frontRightMotor = hardwareMap.dcMotor.get("motor1");
        backLeftMotor = hardwareMap.dcMotor.get("motor2");
        backRightMotor = hardwareMap.dcMotor.get("motor3");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double x, double y, double rx) {
        rx=-rx;
        double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
        frontLeftMotor.setPower(((y + x + rx) / denominator));
        backLeftMotor.setPower(((y - x + rx) / denominator));
        frontRightMotor.setPower(((y - x - rx) / denominator));
        backRightMotor.setPower(((y + x - rx) / denominator));
        this.x = x;
        this.y = y;
    }
    public void testStrafe(double power) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
    }
    public void testDrive(double power) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
    }
    public int fLMPos(){
        return frontLeftMotor.getCurrentPosition();
    }
    public int bLMPos(){
        return backLeftMotor.getCurrentPosition();
    }
    public int fRMPos(){
        return frontRightMotor.getCurrentPosition();
    }
    public int bRMPos(){
        return backRightMotor.getCurrentPosition();
    }
    public void reset(boolean options){
        if (options){
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void getStatus(TelemetryPacket packet) {
        packet.put("FLPower", frontLeftMotor.getPower());
        packet.put("FRPower", frontRightMotor.getPower());
        packet.put("BLPower", backLeftMotor.getPower());
        packet.put("BRPower", backRightMotor.getPower());
        packet.put("XPower", x);
        packet.put("YPower", y);
    }
}
