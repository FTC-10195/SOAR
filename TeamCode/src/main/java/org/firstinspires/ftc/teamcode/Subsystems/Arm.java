package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Arm {
    public enum Extendo {
        RETRACTED,
        EXTENDED,
        CHAMBER,
        BUCKET,
        BUCKET_SECONDARY
    }

    public enum Shoulder {
        INIT,//Upwards is what it will do during resting
        UPWARDS,//Upwards is what it will do during resting
        FORWARDS, //Parallel to the ground
        DOWNWARDS, //Used for intaking only
        CHAMBER_SCORE,
        CHAMBER_SCORE_CPE,
        BACKWARDS,
        CHAMBER_INTAKE,
        BUCKET,
        BUCKET_SECONDARY,
        UNSCORE,
        DEFENSE,
    }

    public enum Wrist {
        CHAMBER_SLIDE_DEPOSIT,
        UNSCORE,
        FORWARD, //Used for most tasks
        DOWNWARDS, //Used for intaking
        FULL_DOWNWARDS, //USED FOR SCOUTING ONLY
        UPWARDS,
        CHAMBER_SCORE_CPE,
    }

    public enum Intake {
        INTAKE,
        DEPOSIT,
        CLOSE
    }

    public enum ClawRotation {
        Vert,
        LEFTDIAG,
        Horz1,
        RIGHTDIAG,
        Horz2,
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
    public static double extendoRetractedPos = .22;
    public static double extendoExtendedPos = 0;
    public static double extendoBucketPos = 0.02;
    public static double extendoBucketSecondaryPos = 0.1;
    public static double extendoChamberPos = .22;
    public static double wristForwardPos = 0.45; //Should be facing straight forwards
    public static double wristUnscorePos = 0.15;
    public static double wristChamberSlideDepositPos = 0.55; //Used for spec auto
    public static double wristDownwardsPos = 0.7; //Should be facing towards the ground, only for INTAKING
    public static double wristFullDownwardsPos = 0.74; //Should be facing towards the ground SCOUTING, CHAMBER SCORE
    public static double wristUpwardsPos = 0.1; //Should be facing the ceiling
    public static double wristChamberScoreCPE = 0.2;
    //Shoulder Positions:
    public static double shoulderInit = .22;
    public static double shoulderDefense = .3;
    public static double shoulderChamberIntake = 0.13;
    public static double shoulderBucket = .3;
    public static double shoulderBucketSecondary = .2;
    public static double shoulderBackwards = .13;
    public static double shoulderUpwards = 0.45;
    public static double shoulderForwards = 0.58;   //Should be parallel to the ground
    public static double shoulderChamberScore = 0.55;
    public static double shoulderChamberScoreCPE = 0.58;
    public static double shoulderUnscore = 0.5;
    public static double shoulderDownwards = 0.675;   //Should be low enough to intake
    public static double clawClosed = .39;
    public static double clawOpen = .6;
    public static double clawVert = .75;
    public static double clawDiag1 = .6;
    public static double clawDiag2 = .2;
    public static double clawHorz1 = .4;
    public static double clawHorz2 = 1;
    public double wristOffset = 0;
    public double shoulderOffset = 0;
    public double wristOffsetGain = .01;
    public double shoulderOffsetGain = .01;

    public long shoulderLerpStartTime = System.currentTimeMillis();
    public static long SHOULDER_LERP_TIME_IN_MILLIS = 400;

    public void setShoulderLerpStartTime(long time) {
        shoulderLerpStartTime = time;
    }

    public double lerp(double startPos, double endPos) {
        long timePassed = System.currentTimeMillis() - shoulderLerpStartTime;
        double difference = endPos - startPos;

        //Must convert to doubles because it is a decimal while longs are not
        double percentComplete = (double) timePassed / (double) SHOULDER_LERP_TIME_IN_MILLIS;

        if (percentComplete > 1) {
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


    public void update(Telemetry telemetry, TeamColor.Color teamColor) {
        telemetry.addData("IntakeState", this.intakeState);
        telemetry.addData("Extendo", this.extendoState);
        telemetry.addData("shoulderLerpStartTime", shoulderLerpStartTime);
        switch (shoulderState) {
            case DOWNWARDS:
                rightShoulder.setPosition(lerp(shoulderForwards + shoulderOffset, shoulderDownwards + shoulderOffset));
                break;
            case UPWARDS:
                rightShoulder.setPosition(shoulderUpwards + shoulderOffset);
                break;
            case FORWARDS:
                rightShoulder.setPosition(shoulderForwards + shoulderOffset);
                break;
            case BACKWARDS:
                rightShoulder.setPosition(shoulderBackwards + shoulderOffset);
                break;
            case INIT:
                rightShoulder.setPosition(shoulderInit + shoulderOffset);
                break;
            case BUCKET:
                rightShoulder.setPosition(shoulderBucket + shoulderOffset);
                break;
            case BUCKET_SECONDARY:
                rightShoulder.setPosition(shoulderBucketSecondary + shoulderOffset);
                break;
            case CHAMBER_INTAKE:
                rightShoulder.setPosition(shoulderChamberIntake + shoulderOffset);
                break;
            case CHAMBER_SCORE:
                rightShoulder.setPosition(shoulderChamberScore + shoulderOffset);
                break;
            case UNSCORE:
                rightShoulder.setPosition(wristUnscorePos + shoulderOffset);
                break;
            case DEFENSE:
                rightShoulder.setPosition(shoulderDefense + shoulderOffset);
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
            case BUCKET:
                extendoServo.setPosition(extendoBucketPos);
                break;
            case BUCKET_SECONDARY:
                extendoServo.setPosition(extendoBucketSecondaryPos);
                break;

        }
        switch (wristState) {
            case FORWARD:
                wristServo.setPosition(wristForwardPos + wristOffset);
                break;
            case CHAMBER_SLIDE_DEPOSIT:
                wristServo.setPosition(wristChamberSlideDepositPos + wristOffset);
                break;
            case DOWNWARDS:
                wristServo.setPosition(wristDownwardsPos + wristOffset);
                break;
            case FULL_DOWNWARDS:
                wristServo.setPosition(wristFullDownwardsPos + wristOffset);
                break;
            case UPWARDS:
                wristServo.setPosition(wristUpwardsPos + wristOffset);
                break;
            case UNSCORE:
                wristServo.setPosition(wristUnscorePos + wristOffset);
                break;
            case CHAMBER_SCORE_CPE:
                wristServo.setPosition(wristChamberScoreCPE + wristOffset);
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
        switch (clawRotation) {
            case Vert:
                clawRotationServo.setPosition(clawVert);
                break;
            case LEFTDIAG:
                clawRotationServo.setPosition(clawDiag1);
                break;
            case Horz1:
                clawRotationServo.setPosition(clawHorz1);
                break;
            case Horz2:
                clawRotationServo.setPosition(clawHorz2);
                break;
            case RIGHTDIAG:
                clawRotationServo.setPosition(clawDiag2);
                break;
        }
        telemetry.addData("WristOffset", wristOffset);
        telemetry.addData("ShoulderOffset", shoulderOffset);
    }


    public boolean isGrabbed() {
        return true;
    }

    public Action updateAction(Telemetry telemetry, TeamColor.Color teamColor) {
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
