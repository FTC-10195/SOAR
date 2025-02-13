package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.opencv.core.Mat;

@Autonomous
public class OdoChamber extends LinearOpMode {
    //MEASUREMENTS
    double ROBOT_LENGTH = 17.1;
    double ROBOT_WIDTH = 16.38;
    double TILE_SIZE = 24;
    double BARRIER_OFFSET = 0; //Probably not necessary
    double EXTENDO_REACH = 2; //Used as an offset
    //Positions
    Vector2d beginVec = new Vector2d (0,0);
    double beginRot = 0;
    Pose2d beginPose = new Pose2d(beginVec , Math.toRadians(beginRot)); //Robot starts facing forwards towards the submersible

    //Pre-score
    Vector2d preScoreVec = new Vector2d(beginVec.x+ (TILE_SIZE - ROBOT_LENGTH/2),beginVec.y); //In between row 1 and row 2
    double preScoreRot = 0;
    Pose2d preScorePos = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
    Vector2d scoreVec = new Vector2d(preScoreVec.x+ 13, preScoreVec.y); //drives forward enough to score
    double scoreRot = preScoreRot;
    Pose2d scorePos = new Pose2d(scoreVec, Math.toRadians(scoreRot));
    Pose2d scorePos2 = new Pose2d(new Vector2d(preScoreVec.x+ 13, preScoreVec.y + 4), Math.toRadians(scoreRot));
    Pose2d scorePos3 = new Pose2d(new Vector2d(preScoreVec.x+ 13, preScoreVec.y + 8), Math.toRadians(scoreRot));
    Vector2d prePushVec1 = new Vector2d(preScoreVec.x, preScoreVec.y - TILE_SIZE + 3); //drives left enough to avoid hitting the submersible
    double prePushRot1 = -90;
    Pose2d prePushPos1 = new Pose2d(prePushVec1, Math.toRadians(prePushRot1));
    Vector2d prePushVec2 = new Vector2d(prePushVec1.x + (TILE_SIZE + (TILE_SIZE/2)) + 2, prePushVec1.y); //drives forward enough, ex 36 inches, to push the samples
    double prePushRot2 = -90;
    Pose2d prePushPos2 = new Pose2d(prePushVec2, Math.toRadians(prePushRot2));
    Vector2d prePushVec3 = new Vector2d(prePushVec2.x, prePushVec2.y - 5); //drives above the first sample
    double prePushRot3 = -90;
    Pose2d prePushPos3 = new Pose2d(prePushVec3, Math.toRadians(prePushRot3));
    Vector2d pushVec1 = new Vector2d(prePushVec3.x- 32 , prePushVec3.y); //pushes first sample
    double pushRot1 = -90;
    Pose2d pushPos1 = new Pose2d(pushVec1, Math.toRadians(pushRot1));

    Vector2d prePushVec4 = new Vector2d(prePushVec3.x, prePushVec3.y-12); //drives above the second sample
    double prePushRot4 = -90;
    Pose2d prePushPos4 = new Pose2d(prePushVec4, Math.toRadians(prePushRot4));

    Vector2d pushVec2 = new Vector2d(prePushVec4.x- 32 , prePushVec4.y); //Pushes second sample
    double pushRot2 = -90;
    Pose2d pushPos2 = new Pose2d(pushVec2, Math.toRadians(pushRot2));
    Vector2d intakeVec1 = new Vector2d(pushVec2.x - 14 , pushVec2.y); //drives to intake
    double intakeRot1 = 0;
    Pose2d intakePos1 = new Pose2d(new Vector2d(intakeVec1.x-5,intakeVec1.y), Math.toRadians(intakeRot1));
Vector2d intakeVec2 = new Vector2d(3 , -TILE_SIZE - 5); //picks it up
    double intakeRot2 = 0;
    Pose2d intakePos2 = new Pose2d(new Vector2d(intakeVec2.x-5,intakeVec2.y), Math.toRadians(0));
    double chamberNumber = -4; //Used as an offset for each new chamber score
    public Action chamber(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                    arm.shoulderAction(Arm.Shoulder.FORWARDS),
                    arm.clawRotationAction(Arm.ClawRotation.Horz1),
                    arm.intakeAction(Arm.Intake.CLOSE),
                    arm.wristAction(Arm.Wrist.DOWNWARDS),
                    arm.extendoAction(Arm.Extendo.RETRACTED),
                    new SleepAction(.4), //Allow the mechanisms to be in the correct positions before raising slides
                    verticalSlides.slideAction(VerticalSlides.SlidePositions.CHAMBER)
                )
        );
    }
    public Action humanIntake(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN),
                        arm.extendoAction(Arm.Extendo.RETRACTED),
                        arm.shoulderAction(Arm.Shoulder.CHAMBER_INTAKE),
                        arm.wristAction(Arm.Wrist.FORWARD),
                        arm.clawRotationAction(Arm.ClawRotation.Horz1)
                )
        );
    }
    private Action toScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos, double num,double tangent){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(tangent))
                                .splineToConstantHeading(new Vector2d(preScoreVec.x,preScoreVec.y + num),Math.toRadians(0))
                                .setTangent(Math.toRadians(scoreRot))
                                .splineToConstantHeading(new Vector2d(scoreVec.x,scoreVec.y + num), Math.toRadians(scoreRot),
                                        ((pose2dDual, posePath, v) ->  30),
                                        ((pose2dDual, posePath, v) -> new MinMax(-30,30)))
                                .build(),
                        arm.intakeAction(Arm.Intake.DEPOSIT),
                        new SleepAction(.2)

                );
    }
    private Action toPushBot(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos, double tangent){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(prePushVec1,Math.toRadians(prePushRot1)),Math.toRadians(-90))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(prePushVec2,Math.toRadians(0))
                                 .setTangent(Math.toRadians(prePushRot3))
                                 .splineToConstantHeading(prePushVec3,Math.toRadians(180))
                                  .setTangent(Math.toRadians(180))
                                  .splineToConstantHeading(pushVec1,Math.toRadians(0))
                                   .setTangent(Math.toRadians(0))
                                   .splineToConstantHeading(prePushVec3,Math.toRadians(0))
                                   .setTangent(Math.toRadians(0))
                                   .splineToConstantHeading(prePushVec4,Math.toRadians(180))
                                   .setTangent(Math.toRadians(180))
                                   .splineToConstantHeading(pushVec2,Math.toRadians(0))
                                .build()

                );
    }
    private Action toIntake(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .turnTo(0)
                                .setTangent(180)
                                .splineToConstantHeading(intakeVec1, Math.toRadians(180))
                                        .build(),
                        //Gives human player some time to correct
                        drive.actionBuilder(intakePos1)
                                .splineToConstantHeading(new Vector2d(intakeVec1.x - 5, intakeVec1.y), Math.toRadians(180),
                                        ((pose2dDual, posePath, v) ->  30),
                                        ((pose2dDual, posePath, v) -> new MinMax(-30,30)))
                                .build(),
                        new SleepAction(.2),
                        arm.intakeAction(Arm.Intake.CLOSE)


                );
    }
    private Action toIntake2(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(180)
                                .splineToConstantHeading(intakeVec2, Math.toRadians(180))
                                .build(),
                        //Gives human player some time to correct
                        drive.actionBuilder(intakePos2)
                                .splineToConstantHeading(new Vector2d(intakeVec2.x - 5, intakeVec2.y), Math.toRadians(180),
                                        ((pose2dDual, posePath, v) ->  30),
                                        ((pose2dDual, posePath, v) -> new MinMax(-30,30)))
                                .build(),
                        new SleepAction(.2),
                        arm.intakeAction(Arm.Intake.CLOSE)


                );
    }
    private Action toPark(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        arm.intakeAction(Arm.Intake.INTAKE),
                        arm.shoulderAction(Arm.Shoulder.INIT),
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN),
                        drive.actionBuilder(pos)
                                .setTangent(180)
                                .splineToConstantHeading(preScoreVec, Math.toRadians(180))
                                .setTangent(-90)
                                .splineToConstantHeading(intakeVec2, Math.toRadians(-90))
                                .build()
                );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm();
        arm.initiate(hardwareMap);
        waitForStart();
        arm.shoulder(Arm.Shoulder.FORWARDS);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // Main Auto functions
                            new SequentialAction(
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                    new SleepAction(.3),
                                    toScore(arm,drive,verticalSlides,beginPose,0,0),
                                    humanIntake(arm,verticalSlides),
                                    toPushBot(arm,drive,verticalSlides,scorePos,0),
                                    toIntake(arm,drive,verticalSlides,pushPos2),
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                    toScore(arm,drive,verticalSlides,intakePos2,4,90),
                                    humanIntake(arm,verticalSlides),
                                    toIntake2(arm,drive,verticalSlides,scorePos2),
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                    toScore(arm,drive,verticalSlides,intakePos2,8,90),
                                    toPark(arm,drive,verticalSlides,scorePos3)
                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
