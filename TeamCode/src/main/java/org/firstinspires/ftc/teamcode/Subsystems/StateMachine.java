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
        SAMPLE_LOADED,
        //Bucket modes
        BUCKET,
        BUCKET_DEPOSIT,
        //Chamber modes
        CHAMBER_READY_TO_FIRE,
        CHAMBER_HUMAN_DEPOSIT,
        CHAMBER_HUMAN_INTAKE,
        CHAMBER,
        CHAMBER_DEPOSIT

    }

    public States updateState(States state, Mode mode, boolean RT, boolean LT, boolean RB, boolean LB,boolean Grabbed, Telemetry telemetry){
        States newState = state;
        if (RT) {
            if (mode == Mode.CHAMBER) {
                if (state == States.RESTING) {
                    newState = States.CHAMBER;
                } else if (state == States.CHAMBER) {
                    newState = States.CHAMBER_DEPOSIT;
                } else if (state == States.CHAMBER_DEPOSIT) {
                    newState = States.RESTING;
                }
            } else {
                if (state == States.RESTING) {
                    newState = States.BUCKET;
                } else if (state == States.BUCKET) {
                    newState = States.BUCKET_DEPOSIT;
                } else if (state == States.BUCKET_DEPOSIT) {
                    newState = States.RESTING;
                }
            }

        }
        if (LB) {
            if (state == States.SAMPLE_LOADED) {
                if (mode == Mode.CHAMBER) {
                    newState = States.CHAMBER_READY_TO_FIRE;
                } else {
                    newState = States.RESTING;
                }
            } else if (state == States.RESTING) {
                newState = States.SCOUTING;
            }else if (state == States.SCOUTING) {
                newState = States.SAMPLE_INTAKE;
            }else if (state == States.SAMPLE_INTAKE){
                newState = States.SCOUTING;
            } else if (state == States.CHAMBER_READY_TO_FIRE) {
                newState = States.CHAMBER_HUMAN_DEPOSIT;
            } else if (state == States.CHAMBER_HUMAN_DEPOSIT) {
                newState = States.SCOUTING;
            }
        }
        if (LT) {
            if (mode == Mode.CHAMBER) {
                if (state == States.RESTING) {
                    newState = States.CHAMBER_HUMAN_INTAKE;
                }
            }
        }
        if (RB) {
            newState = States.RESTING;
        }
        if (Grabbed) {
            if (state == States.SAMPLE_INTAKE) {
                newState = States.SAMPLE_LOADED;
            } else if (state == States.CHAMBER_HUMAN_INTAKE) {
                newState = States.CHAMBER;
            }
        }
        telemetry.addData("Mode", mode);
        telemetry.addData("State", state);
        return newState;
    }
    public  Mode switchMode(Mode mode, boolean Switch){
        if (Switch) {
            if (mode == Mode.CHAMBER) {
                return Mode.BUCKET;
            }
            if (mode == Mode.BUCKET){
                return Mode.CHAMBER;
            }
        }
        return mode;
    }
}
