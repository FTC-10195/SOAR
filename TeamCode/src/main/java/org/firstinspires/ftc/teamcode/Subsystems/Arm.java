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
        EXTENDED,
        CHAMBER
    }

    public enum Shoulder {
        INIT,//Upwards is what it will do during resting
        UPWARDS,//Upwards is what it will do during resting
        FORWARDS, //Parallel to the ground
        DOWNWARDS, //Used for intaking only
        BACKWARDS,
        CHAMBER_INTAKE,
        BUCKET,
    }

    public enum Wrist {
        FORWARD, //Used for most tasks
        DOWNWARDS, //Used for intaking
        FULL_DOWNWARDS, //USED FOR SCOUTING ONLY
        UPWARDS, //Used for scoring chamber
    }

    public enum Intake {
        INTAKE,
        DEPOSIT,
        CLOSE
    }

    public enum ClawRotation {
        Vert,
        Diag1,
        Horz1,
        Diag2,
        Horz2,
    }

    public enum TeamColor {
        RED,
        BLUE,
        NONE
    }

    public Shoulder shoulderState = Shoulder.UPWARDS;
    public Wrist wristState = Wrist.DOWNWARDS;
    public Extendo extendoState = Extendo.RETRACTED;
    public Intake intakeState = Intake.CLOSE;
    public ClawRotation clawRotation = ClawRotation.Horz1;
    Servo claw;
    Servo clawRotationServo;
    Servo extendoServo;
    Servo wristServo;
    Servo rightShoulder; //Dominant servo
    Servo leftShoulder; //Copys rightShoulder
    public static double extendoRetractedPos = .49;
    public static double extendoExtendedPos = .24;
    public static double extendoChamberPos = .3;
    public static double wristForwardPos = 0.42; //Should be facing straight forwards
    public static double wristDownwardsPos = 0.1; //Should be facing towards the ground
    public static double wristFullDownwardsPos = 0.1; //Should be facing towards the ground
    public static double wristUpwardsPos = 0.9; //Should be facing the ceiling
    //Shoulder Positions:
    public static double shoulderInit = .15;
    public static double shoulderChamberIntake = 0.09;
    public static double shoulderBucket = .3;
    public static double shoulderBackwards = .1;
    public static double shoulderUpwards = 0.4;
    public static double shoulderForwards = 0.52;   //Should be parallel to the ground
    public static double shoulderDownwards = 0.625;   //Should be low enough to intake
    public static double clawClosed = .39;
    public static double clawOpen = .6;
    public static double clawVert = .75;
    public static double clawDiag1 = .6;
    public static double clawDiag2 = .2;
    public static double clawHorz1 = .4;
    public static double clawHorz2 = 0;

    public long shoulderLerpStartTime = System.currentTimeMillis();
    public static long SHOULDER_LERP_TIME_IN_MILLIS = 400;
    public void setShoulderLerpStartTime(long time){
        shoulderLerpStartTime = time;
    }

    public double lerp(double startPos, double endPos){
        long timePassed = System.currentTimeMillis() - shoulderLerpStartTime;
        double difference = endPos - startPos;

        //Must convert to doubles because it is a decimal while longs are not
        double percentComplete = (double) timePassed/ (double) SHOULDER_LERP_TIME_IN_MILLIS;

        if (percentComplete > 1){
            percentComplete = 1;
        }
        //Lerp it!
        double currentPos = (difference * percentComplete) + startPos;
        return currentPos;
    }

    public void initiate(HardwareMap hardwareMap) {
        extendoServo = hardwareMap.servo.get("Extendo");
        extendoServo.setPosition(extendoRetractedPos);
        claw = hardwareMap.servo.get("claw");
        clawRotationServo = hardwareMap.servo.get("clawRot");
        clawRotationServo.setPosition(clawVert);
        rightShoulder = hardwareMap.servo.get("RightShoulder");
        leftShoulder = hardwareMap.servo.get("LeftShoulder");
        rightShoulder.setPosition(shoulderInit);
        leftShoulder.setPosition(1 - shoulderInit);
        wristServo = hardwareMap.servo.get("Wrist");
        wristServo.setPosition(wristUpwardsPos);
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

    public void clawRotate(ClawRotation clawRotation) {
        this.clawRotation = clawRotation;
    }

    public TeamColor switchColor(TeamColor teamColor, boolean Switch) {
        TeamColor newTeamColor = teamColor;
        if (Switch) {
            if (teamColor == TeamColor.BLUE) {
                newTeamColor = TeamColor.RED;
            } else {
                newTeamColor = TeamColor.BLUE;
            }
        }
        return newTeamColor;
    }

    public void update(Telemetry telemetry, TeamColor teamColor) {
        telemetry.addData("IntakeState", this.intakeState);
        telemetry.addData("Extendo", this.extendoState);
        telemetry.addData("shoulderLerpStartTime", shoulderLerpStartTime);
        switch (shoulderState) {
            case DOWNWARDS:
                rightShoulder.setPosition(lerp(shoulderForwards,shoulderDownwards));
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
            case INIT:
                rightShoulder.setPosition(shoulderInit);
                break;
            case BUCKET:
                rightShoulder.setPosition(shoulderBucket);
                break;
            case CHAMBER_INTAKE:
                rightShoulder.setPosition(shoulderChamberIntake);
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
            case CHAMBER:
                extendoServo.setPosition(extendoChamberPos);
                break;
        }
        switch (wristState) {
            case FORWARD:
                wristServo.setPosition(wristForwardPos);
                break;
            case DOWNWARDS:
                wristServo.setPosition(wristDownwardsPos);
                break;
            case FULL_DOWNWARDS:
                wristServo.setPosition(wristFullDownwardsPos);
                break;
            case UPWARDS:
                wristServo.setPosition(wristUpwardsPos);
                break;
        }
        switch (intakeState) {
            case INTAKE:
                claw.setPosition(clawOpen);
                break;
            case DEPOSIT:
                claw.setPosition(clawOpen);
                break;
            case CLOSE:
                claw.setPosition(clawClosed);
                break;
        }
        intakeState = checkColor(teamColor, telemetry);
        switch (clawRotation) {
            case Vert:
                clawRotationServo.setPosition(clawVert);
                break;
            case Diag1:
                clawRotationServo.setPosition(clawDiag1);
                break;
            case Horz1:
                clawRotationServo.setPosition(clawHorz1);
                break;
            case Horz2:
                clawRotationServo.setPosition(clawHorz2);
                break;
            case Diag2:
                clawRotationServo.setPosition(clawDiag2);
                break;
        }
        telemetry.addData("Wrist", wristState);
        telemetry.addData("ClawState", intakeState);
        telemetry.addData("ClawRotation", clawRotation);
    }

    public Intake checkColor(TeamColor teamColor, Telemetry telemetry) {
        return intakeState;
    }

    public boolean isGrabbed() {
        return true;
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

    public Action clawRotationAction(ClawRotation state) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawRotate(state);
                return false;
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
    public Action setTimeSnapshot(long time) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setShoulderLerpStartTime(time);
                return false;
            }
        };
    }
    public boolean isLerpComplete() {
        return System.currentTimeMillis() - shoulderLerpStartTime > SHOULDER_LERP_TIME_IN_MILLIS;
    }

    public Action bucketAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder(Shoulder.UPWARDS);
                extendo(Extendo.RETRACTED);
                wrist(Wrist.FORWARD);
                intake(Intake.CLOSE);
                return false;
            }
        };
    }

}
