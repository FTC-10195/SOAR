package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public enum Extendo{
        RETRACTED,
        EXTENDED
    }
    public enum Shoulder{
        STARTING,
        RESTING,
        DOWN
    }
    public enum Wrist {
        FORWARD,
        SIDEWAYS
    }
    public enum Intake {
        INTAKING,
        OUTTAKING,
        STOPPED
    }
    Shoulder shoulderState = Shoulder.RESTING;
    Wrist wristState = Wrist.FORWARD;
    Extendo extendoState = Extendo.RETRACTED;
    Intake intakeState = Intake.STOPPED;
    CRServo intakeServo;
    Servo extendoServo;
    Servo wristServo;
    Servo rightServo;
    Servo leftServo;
    double restPos = 0.1;
    double extendoExtendedPos =.43;
    double wristForwardPos = 0.5;
    double wristSidewaysPos = .83;
    double extendoRetractedPos = .15;
    double shoulderDownOffset = .392;
    double startOffset = -.1;
    //returns a sin out value (from 0 to 1)
    public double easeOutSine(double x){
        return Math.sin((x * Math.PI) / 2);
    }


    public void initiate(HardwareMap hardwareMap){
        extendoServo = hardwareMap.servo.get("Arm Servo");
        extendoServo.setPosition(extendoExtendedPos);
        intakeServo = hardwareMap.crservo.get("Intake");
        rightServo = hardwareMap.servo.get("Right Servo");
        leftServo = hardwareMap.servo.get("Left Servo");
        rightServo.setPosition(restPos);
        leftServo.setPosition(1-restPos);
        wristServo = hardwareMap.servo.get("Wrist");
        wristServo.setPosition(wristForwardPos);
    }
    public void shoulder(Shoulder shoulderState){
        this.shoulderState = shoulderState;
    }
    public void extendo(Extendo extendoState){
        this.extendoState = extendoState;
    }
    public void wrist(Wrist wristState){
        this.wristState = wristState;
    }
    public void intake(Intake intakeState){
        this.intakeState = intakeState;
    }
    public void update(){
        switch (shoulderState){
            case DOWN:
                rightServo.setPosition(restPos + shoulderDownOffset);
                break;
            case RESTING:
                rightServo.setPosition(restPos);
                break;
            case STARTING:
                rightServo.setPosition(restPos + startOffset);
                break;
        }
        leftServo.setPosition(1-rightServo.getPosition());
        switch (extendoState){
            case EXTENDED:
                extendoServo.setPosition(extendoExtendedPos);
                break;
            case RETRACTED:
                extendoServo.setPosition(extendoRetractedPos);
                break;
        }
        switch (wristState){
            case FORWARD:
                wristServo.setPosition(wristForwardPos);
                break;
            case SIDEWAYS:
                wristServo.setPosition(wristSidewaysPos);
                break;
        }
        switch (intakeState){
            case INTAKING:
                intakeServo.setPower(-1);;
                break;
            case OUTTAKING:
                intakeServo.setPower(1);;
                break;
            case STOPPED:
                intakeServo.setPower(0);;
                break;
        }

    }
    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return true;
            }
        };
    }
    public Action extendoAction(Extendo state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                extendo(state);
                return false;
            }
        };
    }
    public Action wristAction(Wrist state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist(state);
                return false;
            }
        };
    }
    public Action intakeAction(Intake state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake(state);
                return false;
            }
        };
    }
    public Action shoulderAction(Shoulder state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder(state);
                return false;
            }
        };
    }
    public Action bucketAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder(Shoulder.RESTING);
                extendo(Extendo.RETRACTED);
                wrist(Wrist.FORWARD);
                intake(Intake.STOPPED);
                return false;
            }
        };
    }

}
