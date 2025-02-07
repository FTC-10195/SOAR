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
            switch (state){
                case RESTING:
                    newState = States.SCOUTING;
                     break;
                case SCOUTING:
                    newState = States.SAMPLE_INTAKE;
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
            if (mode != Mode.CHAMBER){
                return null;
            }
            switch (state){
                case RESTING:
                    newState = States.CHAMBER_HUMAN_INTAKE;
                    break;
                case CHAMBER_HUMAN_INTAKE:
                    newState = States.CHAMBER;
                    break;
                case CHAMBER:
                    newState = States.RESTING;
                    break;
            }
        }
        if (RB) {
            newState = States.RESTING;
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
