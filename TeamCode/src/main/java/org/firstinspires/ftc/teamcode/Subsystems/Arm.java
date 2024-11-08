package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Arm {
    CRServo intakeServo;
    Servo extendoServo;
    Servo wristServo;
    Servo rightServo;
    Servo leftServo;
    double rightStartPos = 0.1;
    double leftStartPos = .9;
    double extendoStartPos =.5;
    double wristStartPos = 0.5;
    public void initiate(HardwareMap hardwareMap){
        extendoServo = hardwareMap.servo.get("Arm Servo");
        extendoServo.setPosition(extendoStartPos);
        intakeServo = hardwareMap.crservo.get("Intake");
        rightServo = hardwareMap.servo.get("Right Servo");
        leftServo = hardwareMap.servo.get("Left Servo");
        rightServo.setPosition(rightStartPos);
        leftServo.setPosition(leftStartPos);
        wristServo = hardwareMap.servo.get("Wrist");
        wristServo.setPosition(wristStartPos);
    }
    public void shoulder(boolean resting, Telemetry telemetry){
        double targetShoulderPos = rightStartPos;
        if (!resting){
            targetShoulderPos = .4;
        }
        rightServo.setPosition(rightStartPos + targetShoulderPos);
        leftServo.setPosition(leftStartPos - targetShoulderPos);
        telemetry.addData("currentArmPos", rightServo.getPosition());
    }
    public void extendo(boolean retracted){
        if (retracted){
            extendoServo.setPosition(extendoStartPos);
        } else {
            extendoServo.setPosition(.75);
        }
    }
    public void wrist(boolean forward){
        if (forward) {
            wristServo.setPosition(wristStartPos);
        } else {
            wristServo.setPosition(.83);
        }
    }
    public void intake(double power){
        intakeServo.setPower(power);
    }
}
