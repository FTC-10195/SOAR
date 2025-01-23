package org.firstinspires.ftc.teamcode.Subsystems;

public class StateMachine {
    public enum States {
        RESTING,
        INTAKING,
        INTAKE_STOP,
        BUCKET,
        BUCKET_DEPOSIT,
        CHAMBER,
        CHAMBER_DEPOSIT

    }
    Arm arm = new Arm();
    VerticalSlides verticalSlides = new VerticalSlides();
    public VerticalSlides.SlidePositions slidePosition = VerticalSlides.SlidePositions.DOWN;
   public void update(States state) {
       switch (state) {
           case RESTING:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.RESTING);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case INTAKING:
               arm.extendo(Arm.Extendo.EXTENDED);
               arm.shoulder(Arm.Shoulder.DOWN);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.INTAKING);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case INTAKE_STOP:
               arm.extendo(Arm.Extendo.EXTENDED);
               arm.shoulder(Arm.Shoulder.DOWN);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
           case BUCKET:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.RESTING);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
               break;
           case BUCKET_DEPOSIT:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.RESTING);
               arm.wrist(Arm.Wrist.FORWARD);
               arm.intake(Arm.Intake.OUTTAKING);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
               break;
           case CHAMBER:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.RESTING);
               arm.wrist(Arm.Wrist.SIDEWAYS);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
               break;
           case CHAMBER_DEPOSIT:
               arm.extendo(Arm.Extendo.RETRACTED);
               arm.shoulder(Arm.Shoulder.RESTING);
               arm.wrist(Arm.Wrist.SIDEWAYS);
               arm.intake(Arm.Intake.STOPPED);
               verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
               break;
       }
   }
}
