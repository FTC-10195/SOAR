package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StateMachine {
    public enum Mode {
        CHAMBER,
        BUCKET,
    }

    public enum States {
        RESTING,
        SCOUTING,
        SAMPLE_INTAKE,
        //Bucket modes
        BUCKET,
        //Chamber modes
        CHAMBER_PRE_DEPOSIT,
        CHAMBER_HUMAN_INTAKE,
        CHAMBER,
        CHAMBER_DEPOSIT
    }
    public long timeSnapshot = System.currentTimeMillis();
    public Arm.Intake clawState = Arm.Intake.INTAKE.CLOSE;
    public void setClawState(Arm.Intake newClawState){
        clawState = newClawState;
    }
    public States setState(States state, Mode mode, boolean RT, boolean LT, boolean RB, boolean LB, Telemetry telemetry) {
        States newState = state;
        if (RT) {
            if (state == States.SAMPLE_INTAKE || state == States.SCOUTING){
                if (clawState == Arm.Intake.CLOSE){
                    clawState = Arm.Intake.INTAKE;
                }else{
                    clawState = Arm.Intake.CLOSE;
                }
            }
            if (mode == Mode.CHAMBER) {
                if (state == States.CHAMBER && clawState == Arm.Intake.CLOSE) {
                    clawState = Arm.Intake.INTAKE;
                }else if (state == States.CHAMBER && clawState != Arm.Intake.CLOSE){
                    newState = States.RESTING;
                }else if (state == States.RESTING){
                    newState = States.CHAMBER;
                }
            } else {
                if (state == States.BUCKET && clawState == Arm.Intake.CLOSE) {
                    clawState = Arm.Intake.INTAKE;
                }else if (state == States.BUCKET && clawState != Arm.Intake.CLOSE){
                    newState = States.RESTING;
                }else if (state == States.RESTING){
                    newState = States.BUCKET;
                }
            }

        }
        if (LB) {
            switch (state){
                case RESTING:
                    clawState = Arm.Intake.INTAKE;
                    newState = States.SCOUTING;
                     break;
                case CHAMBER_HUMAN_INTAKE:
                    clawState = Arm.Intake.INTAKE;
                    newState = States.SCOUTING;
                    break;
                case SCOUTING:
                    newState = States.SAMPLE_INTAKE;
                    clawState = Arm.Intake.INTAKE;
                    timeSnapshot = System.currentTimeMillis();
                    break;
                case SAMPLE_INTAKE:
                    newState = States.SCOUTING;
                    break;
                case CHAMBER_PRE_DEPOSIT:
                    newState = States.CHAMBER;
                    break;
                case CHAMBER:
                    newState = States.RESTING;
                    break;
            }
        }
        if (LT) {
            if (state == States.CHAMBER_HUMAN_INTAKE){
                if (clawState == Arm.Intake.CLOSE){
                    clawState = Arm.Intake.INTAKE;
                }else{
                    clawState = Arm.Intake.CLOSE;
                }
            }
            newState = States.CHAMBER_HUMAN_INTAKE;
        }
        if (RB) {
            newState = States.RESTING;
            clawState = Arm.Intake.CLOSE;
        }
        telemetry.addData("Mode", mode);
        telemetry.addData("State", state);
        return newState;
    }
    public Mode switchMode(Mode mode, boolean Switch){
        Mode newMode = mode;
        if (Switch) {
            if (mode == Mode.CHAMBER) {
                newMode = Mode.BUCKET;
            } else {
                newMode = Mode.CHAMBER;
            }
        }
        return  newMode;
    }
}
