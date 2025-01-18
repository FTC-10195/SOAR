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
        BUCKET_DEPOSIT,
        //Chamber modes
        CHAMBER_READY_TO_FIRE,
        CHAMBER_HUMAN_DEPOSIT,
        CHAMBER_HUMAN_INTAKE,
        CHAMBER,
        CHAMBER_DEPOSIT

    }
    Arm arm = new Arm();
    VerticalSlides verticalSlides = new VerticalSlides();
   public void update(States state, Mode mode, boolean RT, boolean LT, boolean RB, boolean LB, boolean Switch, Telemetry telemetry) {
       switch (state) {
           case RESTING:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.UPWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case SCOUTING:
               arm.extendo(Arm.Extendo.EXTENDED);
               arm.shoulder(Arm.Shoulder.FORWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case SAMPLE_INTAKE:
               arm.extendo(Arm.Extendo.EXTENDED);
               arm.shoulder(Arm.Shoulder.DOWNWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.INTAKING);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case BUCKET:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.BACKWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
               break;
           case BUCKET_DEPOSIT:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.BACKWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.OUTTAKING);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
               break;
           case CHAMBER:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.BACKWARDS);
               arm.wrist(Arm.Wrist.UPWARDS);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
               break;
           case CHAMBER_DEPOSIT:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.BACKWARDS);
               arm.wrist(Arm.Wrist.UPWARDS);
               arm.intake(Arm.Intake.OUTTAKING);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
               break;
           case CHAMBER_READY_TO_FIRE:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.BACKWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case CHAMBER_HUMAN_DEPOSIT:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.BACKWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.SHOOTING);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case CHAMBER_HUMAN_INTAKE:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.FORWARDS);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.INTAKING);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
       }
       if (RT) {
           if (mode == Mode.CHAMBER){
               if (state == States.RESTING) {
                   state = States.CHAMBER;
               } else if (state == States.CHAMBER) {
                   state = States.CHAMBER_DEPOSIT;
               } else if (state == States.CHAMBER_DEPOSIT) {
                   state = States.RESTING;
               }
           }else{
               if (state == States.RESTING) {
                   state = States.BUCKET;
               } else if (state == States.BUCKET) {
                   state = States.BUCKET_DEPOSIT;
               } else if (state == States.BUCKET_DEPOSIT) {
                   state = States.RESTING;
               }
           }

       }
       if (LB){
          if (state == States.RESTING){
              state = States.SCOUTING;
          }else if (state == States.SCOUTING){
              state = States.SAMPLE_INTAKE;
          }else if (state == States.CHAMBER_READY_TO_FIRE){
              state = States.CHAMBER_HUMAN_DEPOSIT;
          }
       }
       if (LT){
           if (mode == Mode.CHAMBER) {
               if (state == States.RESTING) {
                   state = States.CHAMBER_HUMAN_INTAKE;
               }
           }
       }
       if (RB){
           state = States.RESTING;
       }
       if (arm.isGrabbed()){
           if (state == States.SAMPLE_INTAKE) {
               if (mode == Mode.CHAMBER) {
                   state = States.CHAMBER_READY_TO_FIRE;
               } else {
                   state = States.RESTING;
               }
           } else if (state == States.CHAMBER_HUMAN_INTAKE){
               state = States.CHAMBER;
           }
       }
       if (Switch){
           if (mode == Mode.CHAMBER){
               mode = Mode.BUCKET;
           }else{
               mode = Mode.CHAMBER;
           }
       }
       telemetry.addData("Mode",mode);
       telemetry.addData("State",state);
   }
}
