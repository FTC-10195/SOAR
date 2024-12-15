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

    public Shoulder shoulderState = Shoulder.RESTING;
    public Wrist wristState = Wrist.FORWARD;
    public Extendo extendoState = Extendo.RETRACTED;
    public Intake intakeState = Intake.STOPPED;
    CRServo intakeServo;
    Servo extendoServo;
    Servo wristServo;
    Servo rightServo;
    Servo leftServo;
    ColorSensor colorSensor;
    public static double restPos = 0;
    public static double extendoExtendedPos = .19;
    double wristForwardPos = 0.5;
    double wristSidewaysPos = .83;
    public static double extendoRetractedPos = .43;
    public static double shoulderDownOffset = .32;
   public static double startOffset = -restPos;

    //returns a sin out value (from 0 to 1)
    public double easeOutSine(double x) {
        return Math.sin((x * Math.PI) / 2);
    }


    public void initiate(HardwareMap hardwareMap) {
        extendoServo = hardwareMap.servo.get("Arm Servo");
        extendoServo.setPosition(extendoRetractedPos);
        intakeServo = hardwareMap.crservo.get("Intake");
        rightServo = hardwareMap.servo.get("Right Servo");
        leftServo = hardwareMap.servo.get("Left Servo");
        rightServo.setPosition(restPos);
        leftServo.setPosition(1 - restPos);
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

    public void update(Telemetry telemetry, String teamColor) {
        switch (shoulderState) {
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
        leftServo.setPosition(1 - rightServo.getPosition());
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
            case SIDEWAYS:
                wristServo.setPosition(wristSidewaysPos);
                break;
        }
        switch (intakeState) {
            case INTAKING:
                intakeServo.setPower(-1);
                ;
                break;
            case OUTTAKING:
                intakeServo.setPower(1);
                ;
                break;
            case STOPPED:
                intakeServo.setPower(0);
                ;
                break;
        }
        if (checkColor(teamColor)){
            intakeServo.setPower(1);
        }
    }
public boolean checkColor(String teamColor){
    int red = colorSensor.red();  // Get the red value
    int green = colorSensor.green();  // Get the green value
    int blue = colorSensor.blue();  // Get the blue value
    if (Objects.equals(teamColor, "blue")) {
        if (red > blue && red > green) {
            return true;
        }
    }else if (Objects.equals(teamColor, "red")){
        if (blue > red && blue > green) {
            return true;
        }
    }
    return false;
}
    public Action updateAction(Telemetry telemetry, String teamColor) {
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
                shoulder(Shoulder.RESTING);
                extendo(Extendo.RETRACTED);
                wrist(Wrist.FORWARD);
                intake(Intake.STOPPED);
                return false;
            }
        };
    }

}
