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
    public static double extendoExtendedPos = .7;
    public static  double wristForwardPos = 0.5; //Should be facing straight forwards
    public static double wristDownwardsPos = 0.1; //Should be facing towards the ground
    public static double wristUpwardsPos = 0.9; //Should be facing the ceiling
    //Shoulder Positions:
    public static double shoulderBackwards = 0;
    public static double shoulderUpwards = 0.2;
    public static double shoulderForwards = 0.4;   //Should be parallel to the ground
    public static double shoulderDownwards = 0.5;   //Should be low enough to intake

    public void initiate(HardwareMap hardwareMap) {
        extendoServo = hardwareMap.servo.get("Arm Servo");
        extendoServo.setPosition(extendoRetractedPos);
        intakeServoRight = hardwareMap.crservo.get("IntakeRight");
        intakeServoLeft = hardwareMap.crservo.get("IntakeLeft");
        rightShoulder = hardwareMap.servo.get("Right Servo");
        leftShoulder = hardwareMap.servo.get("Left Servo");
        rightShoulder.setPosition(shoulderUpwards);
        leftShoulder.setPosition(1 - shoulderUpwards);
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
public void switchColor(TeamColor teamColor,boolean Switch){
        if (Switch){
            if (teamColor == TeamColor.BLUE){
                teamColor = TeamColor.RED;
            }else {
                teamColor = TeamColor.BLUE;
            }
        }
}
    public void update(Telemetry telemetry, TeamColor teamColor) {
        telemetry.addData("IntakeState",this.intakeState);
        switch (shoulderState) {
            case DOWNWARDS:
                rightShoulder.setPosition(shoulderDownwards);
                break;
            case UPWARDS:
                rightShoulder.setPosition(shoulderUpwards);
                break;
            case FORWARDS:
                rightShoulder.setPosition(shoulderForwards);
                break;
            case BACKWARDS:
                rightShoulder.setPosition(shoulderBackwards);
                break;
        }
        leftShoulder.setPosition(1 - rightShoulder.getPosition());
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
    telemetry.addData("teamColor",teamColor);
    if (intakeState == Intake.OUTTAKING){
        return Intake.OUTTAKING;
    }
    if ((teamColor == TeamColor.BLUE && red > blue && red > green) || (teamColor == TeamColor.RED && blue > red && blue > green)){
        return Intake.OUTTAKING;
    }
    if (isGrabbed()) {
        return Intake.STOPPED;
    }
    return this.intakeState;
}
public boolean isGrabbed(){
        return colorSensor.alpha() < 400;
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
