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
        //Chamber modes
        CHAMBER_PRE_DEPOSIT,
        CHAMBER_HUMAN_INTAKE,
        CHAMBER,
        CHAMBER_DEPOSIT
    }

    Arm arm = new Arm();
    VerticalSlides verticalSlides = new VerticalSlides();

    public States setState(States state, Mode mode, boolean RT, boolean LT, boolean RB, boolean LB, Telemetry telemetry) {
        States newState = state;
        if (RT) {
            if (mode == Mode.CHAMBER) {
                if (state == States.RESTING) {
                    newState = States.CHAMBER;
                } else if (state == States.CHAMBER) {
                    newState = States.RESTING;
                }
            } else {
                if (state == States.RESTING) {
                    newState = States.BUCKET;
                } else if (state == States.BUCKET) {
                    newState = States.RESTING;
                }
            }

        }
        if (LB) {
            if (state == States.SAMPLE_LOADED) {
                if (mode == Mode.CHAMBER) {
                    newState = States.CHAMBER_PRE_DEPOSIT;
                } else {
                    newState = States.RESTING;
                }
            } else if (state == States.RESTING) {
                newState = States.SCOUTING;
            } else if (state == States.SCOUTING) {
                newState = States.SAMPLE_INTAKE;
            } else if (state == States.SAMPLE_INTAKE) {
            newState = States.SCOUTING;
        }else if (state == States.CHAMBER_PRE_DEPOSIT) {
                newState = States.CHAMBER;
            }else if (state == States.CHAMBER) {
                newState = States.CHAMBER_DEPOSIT;
            }
        }
        if (LT) {
            if (mode == Mode.CHAMBER) {
                if (state == States.RESTING) {
                    newState = States.CHAMBER_HUMAN_INTAKE;
                }else if (state == States.CHAMBER_HUMAN_INTAKE){
                    newState = States.CHAMBER;
                }else if (state == States.CHAMBER) {
                    newState = States.CHAMBER_DEPOSIT;
                }else if (state == States.CHAMBER_DEPOSIT) {
                    newState = States.RESTING;
                }
            }
        }
        if (RB) {
            newState = States.RESTING;
        }
        if (arm.isGrabbed()) {
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
