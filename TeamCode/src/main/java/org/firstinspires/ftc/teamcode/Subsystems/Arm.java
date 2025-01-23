package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

@Config
public class Arm {
    public enum Extendo {
        RETRACTED,
        EXTENDED
    }

    public enum Shoulder {
        UPWARDS,//Upwards is what it will do during resting
        FORWARDS, //Parallel to the ground
        DOWNWARDS, //Used for intaking only
        BACKWARDS,
    }

    public enum Wrist {
        FORWARD, //Used for most tasks
        DOWNWARDS, //Used for intaking/scouting
        UPWARDS, //Used for scoring chamber
    }

    public enum Intake {
        INTAKING,
        OUTTAKING,
        SHOOTING,
        STOPPED
    }
    public enum TeamColor {
        RED,
        BLUE,
        NONE
    }

    public Shoulder shoulderState = Shoulder.UPWARDS;
    public Wrist wristState = Wrist.FORWARD;
    public Extendo extendoState = Extendo.RETRACTED;
    public Intake intakeState = Intake.STOPPED;
    CRServo intakeServoRight;
    CRServo intakeServoLeft;
    Servo extendoServo;
    Servo wristServo;
    Servo rightShoulder; //Dominant servo
    Servo leftShoulder; //Copys rightShoulder
    ColorSensor colorSensor;
    public static double extendoRetractedPos = .5;
    public static double extendoExtendedPos = .16;
    public static  double wristForwardPos = 0.45; //Should be facing straight forwards
    public static double wristDownwardsPos = 0.9; //Should be facing towards the ground
    public static double wristUpwardsPos = 0.1; //Should be facing the ceiling
    //Shoulder Positions:
    public static double shoulderBackwards = 0.14;
    public static double shoulderUpwards = 0.35;
    public static double shoulderForwards = 0.48;   //Should be parallel to the ground
    public static double shoulderDownwards = 0.5;   //Should be low enough to intake
    public static double leftOffset = -.12;
    public static double rightOffset = 0;
    public void initiate(HardwareMap hardwareMap) {
        extendoServo = hardwareMap.servo.get("Extendo");
        extendoServo.setPosition(extendoRetractedPos);
        intakeServoRight = hardwareMap.crservo.get("RightIntake");
        intakeServoLeft = hardwareMap.crservo.get("LeftIntake");
        rightShoulder = hardwareMap.servo.get("RightShoulder");
        leftShoulder = hardwareMap.servo.get("LeftShoulder");
        rightShoulder.setPosition(shoulderUpwards);
        leftShoulder.setPosition(1 - shoulderUpwards - leftOffset);
        wristServo = hardwareMap.servo.get("Wrist");
        wristServo.setPosition(wristForwardPos);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public void shoulder(Shoulder shoulderState) {
        this.shoulderState = shoulderState;
    }

    public void extendo(Extendo extendoState) {
        this.extendoState = extendoState;
    }

    public void wrist(Wrist wristState) {
        this.wristState = wristState;
    }

    public void intake(Intake intakeState) {
        this.intakeState = intakeState;
    }
public TeamColor switchColor(TeamColor teamColor,boolean Switch){
        if (Switch){
            if (teamColor == TeamColor.BLUE){
                return TeamColor.RED;
            }else {
                return TeamColor.BLUE;
            }
        }
        return teamColor;
}
    public void update(Telemetry telemetry, TeamColor teamColor) {
        telemetry.addData("IntakeState",this.intakeState);
        telemetry.addData("extendoState",extendoState);
        switch (shoulderState) {
            case DOWNWARDS:
                rightShoulder.setPosition(shoulderDownwards+ rightOffset);
                leftShoulder.setPosition(1 - (shoulderDownwards + leftOffset));
                break;
            case UPWARDS:
                rightShoulder.setPosition(shoulderUpwards+ rightOffset);
                leftShoulder.setPosition(1 - (shoulderUpwards + leftOffset));
                break;
            case FORWARDS:
                rightShoulder.setPosition(shoulderForwards+ rightOffset);
                leftShoulder.setPosition(1 - (shoulderForwards + leftOffset));
                break;
            case BACKWARDS:
                rightShoulder.setPosition(shoulderBackwards+ rightOffset);
                leftShoulder.setPosition(1 - (shoulderBackwards + leftOffset));
                break;
        }
        switch (extendoState) {
            case EXTENDED:
                extendoServo.setPosition(extendoExtendedPos);
                break;
            case RETRACTED:
                extendoServo.setPosition(extendoRetractedPos);
                break;
        }
        switch (wristState) {
            case FORWARD:
                wristServo.setPosition(wristForwardPos);
                break;
            case DOWNWARDS:
                wristServo.setPosition(wristDownwardsPos);
                break;
            case UPWARDS:
                wristServo.setPosition(wristUpwardsPos);
                break;
        }
        intakeState = checkColor(teamColor,telemetry);
        switch (intakeState) {
            case INTAKING:
                intakeServoRight.setPower(-1);
                intakeServoLeft.setPower(1);
                break;
            case SHOOTING:
                intakeServoRight.setPower(1);
                intakeServoLeft.setPower(-1);
                break;
            case OUTTAKING:
                intakeServoRight.setPower(.3);
                intakeServoLeft.setPower(-.3);
                break;
            case STOPPED:
                intakeServoRight.setPower(0);
                intakeServoLeft.setPower(0);
                break;
        }
    }
public Intake checkColor(TeamColor teamColor, Telemetry telemetry){
    int red = colorSensor.red();
    int green = colorSensor.green();
    int blue = colorSensor.blue();
    telemetry.addData("blue:",blue);
    telemetry.addData("red:",red);
    telemetry.addData("green:",green);
    telemetry.addData("distance",colorSensor.alpha());
    telemetry.addData("isGrabbed", isGrabbed());
    telemetry.addData("teamColor",teamColor);
    if (intakeState == Intake.OUTTAKING || intakeState == Intake.SHOOTING){
        return this.intakeState;
    }
    if ((teamColor == TeamColor.BLUE && red > blue && red > green && colorSensor.alpha() > 100) || (teamColor == TeamColor.RED && blue > red && blue > green && colorSensor.alpha() > 100)){
        return Intake.SHOOTING;
    }
    if (isGrabbed()) {
        return Intake.STOPPED;
    }
    return this.intakeState;
}
public boolean isGrabbed(){
       if (colorSensor != null){
           return colorSensor.alpha() > 400;
       }else {
           return false;
       }

}
    public Action updateAction(Telemetry telemetry, TeamColor teamColor) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update(telemetry, teamColor);
                return true;
            }
        };
    }

    public Action extendoAction(Extendo state) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                extendo(state);
                return false;
            }
        };
    }

    public Action wristAction(Wrist state) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist(state);
                return false;
            }
        };
    }

    public Action intakeAction(Intake state) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake(state);
                return false;
            }
        };
    }

    public Action shoulderAction(Shoulder state) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder(state);
                return false;
            }
        };
    }

    public Action bucketAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder(Shoulder.UPWARDS);
                extendo(Extendo.RETRACTED);
                wrist(Wrist.FORWARD);
                intake(Intake.STOPPED);
                return false;
            }
        };
    }

}
