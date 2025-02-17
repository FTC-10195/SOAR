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
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
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
    Pose2d preScorePos1 = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
    Pose2d preScorePos2 = new Pose2d(new Vector2d(preScoreVec.x, preScoreVec.y + 3), Math.toRadians(preScoreRot));
    Pose2d preScorePos3 = new Pose2d(new Vector2d(preScoreVec.x, preScoreVec.y + 6), Math.toRadians(preScoreRot));
    Pose2d preScorePos4 = new Pose2d(new Vector2d(preScoreVec.x, preScoreVec.y - 3), Math.toRadians(preScoreRot));
    Vector2d scoreVec = new Vector2d(preScoreVec.x+ 10.5, preScoreVec.y); //drives forward enough to score
    double scoreRot = 0;
    Pose2d scorePos1 = new Pose2d(scoreVec, Math.toRadians(scoreRot));
    Pose2d scorePos2 = new Pose2d(new Vector2d(scoreVec.x, preScorePos2.position.y), Math.toRadians(scoreRot));
    Pose2d scorePos3 = new Pose2d(new Vector2d(scoreVec.x, preScorePos3.position.y), Math.toRadians(scoreRot));
    Pose2d scorePos4 = new Pose2d(new Vector2d(scoreVec.x, preScorePos4.position.y), Math.toRadians(scoreRot));
    Vector2d prePushVec1 = new Vector2d(preScoreVec.x, preScoreVec.y - TILE_SIZE + 3); //drives left enough to avoid hitting the submersible
    double prePushRot1 = -90;
    Pose2d prePushPos1 = new Pose2d(prePushVec1, Math.toRadians(prePushRot1));
    Vector2d prePushVec2 = new Vector2d(prePushVec1.x + (TILE_SIZE + (TILE_SIZE/2)) + 2, prePushVec1.y); //drives forward enough, ex 36 inches, to push the samples
    Vector2d midPushVec = new Vector2d((prePushVec2.x/2) + 3,prePushVec2.y);
    double prePushRot2 = -90;
    Pose2d prePushPos2 = new Pose2d(prePushVec2, Math.toRadians(prePushRot2));
    Vector2d prePushVec3 = new Vector2d(prePushVec2.x, prePushVec2.y - 6); //drives above the first sample
    double prePushRot3 = -90;
    Pose2d prePushPos3 = new Pose2d(prePushVec3, Math.toRadians(prePushRot3));
    Vector2d pushVec1 = new Vector2d(prePushVec3.x- 36 , prePushVec3.y); //pushes first sample
    double pushRot1 = -90;
    Pose2d pushPos1 = new Pose2d(pushVec1, Math.toRadians(pushRot1));

    Vector2d prePushVec4 = new Vector2d(prePushVec3.x, prePushVec3.y-14); //drives above the second sample
    double prePushRot4 = -90;
    Pose2d prePushPos4 = new Pose2d(prePushVec4, Math.toRadians(prePushRot4));

    Vector2d pushVec2 = new Vector2d(prePushVec4.x- 36 , prePushVec4.y); //Pushes second sample
    double pushRot2 = -90;
    Pose2d pushPos2 = new Pose2d(pushVec2, Math.toRadians(pushRot2));
    Vector2d intakeVec1 = new Vector2d(8.5 , pushVec2.y); //drives to intake
    Pose2d intakePosVec = new Pose2d(new Vector2d(intakeVec1.x,intakeVec1.y), Math.toRadians(0));
    double intakeRot1 = 0;
    Pose2d intakePos1 = new Pose2d(new Vector2d(intakeVec1.x-5,intakeVec1.y), Math.toRadians(intakeRot1));
Vector2d intakeVec2 = new Vector2d(intakeVec1.x, -TILE_SIZE - 6.5); //picks it up
    Pose2d intakePosVec2 = new Pose2d(new Vector2d(intakeVec2.x,intakeVec2.y), Math.toRadians(0));
    double intakeRot2 = 0;
    Pose2d intakePos2 = new Pose2d(new Vector2d(intakeVec2.x-5,intakeVec2.y), Math.toRadians(0));
    public Action chamber(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                    arm.shoulderAction(Arm.Shoulder.FORWARDS),
                    arm.clawRotationAction(Arm.ClawRotation.Horz1),
                    arm.intakeAction(Arm.Intake.CLOSE),
                    arm.wristAction(Arm.Wrist.DOWNWARDS),
                    arm.extendoAction(Arm.Extendo.RETRACTED),
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
    private Action toScore1(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos, double num,double tangent){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(tangent))
                                .splineToConstantHeading(preScorePos1.position,Math.toRadians(0))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(scorePos1.position, Math.toRadians(180),
                                        ((pose2dDual, posePath, v) ->  30),
                                        ((pose2dDual, posePath, v) -> new MinMax(-30,30)))
                                .build(),
                        new ParallelAction(
                                arm.intakeAction(Arm.Intake.DEPOSIT),
                                drive.actionBuilder(scorePos1)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(preScorePos1.position,Math.toRadians(-90))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(.1),
                                        humanIntake(arm, verticalSlides)
                                )
                        )
                );
    }
    private Action toScore2(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos, double num,double tangent){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(tangent))
                                .splineToConstantHeading(preScorePos2.position,Math.toRadians(0))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(scorePos2.position, Math.toRadians(180),
                                        ((pose2dDual, posePath, v) ->  30),
                                        ((pose2dDual, posePath, v) -> new MinMax(-30,30)))
                                .build(),
                        new ParallelAction(
                                arm.intakeAction(Arm.Intake.DEPOSIT),
                                drive.actionBuilder(scorePos2)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(preScorePos2.position,Math.toRadians(-90))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(.1),
                                        humanIntake(arm, verticalSlides)
                                )
                        )
                );
    }
    private Action toScore3(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos, double num,double tangent){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(tangent))
                                .splineToConstantHeading(preScorePos3.position,Math.toRadians(0))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(scorePos3.position, Math.toRadians(180),
                                        ((pose2dDual, posePath, v) ->  30),
                                        ((pose2dDual, posePath, v) -> new MinMax(-30,30)))
                                .build(),
                        new ParallelAction(
                                arm.intakeAction(Arm.Intake.DEPOSIT),
                                drive.actionBuilder(scorePos3)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(preScorePos3.position,Math.toRadians(-90))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(.1),
                                        humanIntake(arm, verticalSlides)
                                )
                        )
                );
    }
    private Action toScore4(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos, double num,double tangent){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(tangent))
                                .splineToConstantHeading(preScorePos4.position,Math.toRadians(0))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(scorePos4.position, Math.toRadians(180),
                                        ((pose2dDual, posePath, v) ->  30),
                                        ((pose2dDual, posePath, v) -> new MinMax(-30,30)))
                                .build(),
                        new ParallelAction(
                                arm.intakeAction(Arm.Intake.DEPOSIT),
                                drive.actionBuilder(scorePos4)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(preScorePos4.position,Math.toRadians(-90))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(.1),
                                        humanIntake(arm, verticalSlides)
                                )
                        )
                );
    }
    private Action toPushBot(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos, double tangent){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(-90))
                                .strafeToLinearHeading(prePushVec1,Math.toRadians(prePushRot1))
                                .strafeToConstantHeading(midPushVec)
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(prePushVec2,Math.toRadians(-25))
                                 .setTangent(Math.toRadians(-25))
                                 .splineToConstantHeading(prePushVec3,Math.toRadians(180))
                                  .setTangent(Math.toRadians(180))
                                  .splineToConstantHeading(pushVec1,Math.toRadians(0))
                                   .setTangent(Math.toRadians(0))
                                   .splineToConstantHeading(prePushVec3,Math.toRadians(-90))
                                   .setTangent(Math.toRadians(-90))
                                   .splineToConstantHeading(prePushVec4,Math.toRadians(180))
                                   .setTangent(Math.toRadians(180))
                                   .splineToConstantHeading(pushVec2,Math.toRadians(90))
                                .build()

                );
    }
    private Action toIntake(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(90)
                                .splineToLinearHeading(intakePosVec, Math.toRadians(180))
                                .strafeToConstantHeading(intakePos1.position)
                                .build(),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        new SleepAction(.07)
                );
    }
    private Action toIntake2(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(-90)
                                .splineToConstantHeading(intakePosVec2.position, Math.toRadians(180))
                                .build(),
                        drive.actionBuilder(intakePosVec2)
                                .strafeToConstantHeading(intakePos2.position)
                                .build(),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        new SleepAction(.07)
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
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        waitForStart();
        drive.pinpoint.resetPosAndIMU();
        arm.shoulder(Arm.Shoulder.FORWARDS);
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // Main Auto functions
                            new SequentialAction(
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                    new SleepAction(.7),
                                    toScore1(arm,drive,verticalSlides,beginPose,0,0),
                                    humanIntake(arm,verticalSlides),
                                    toPushBot(arm,drive,verticalSlides,preScorePos1,0),
                                    toIntake(arm,drive,verticalSlides,pushPos2),
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                    toScore2(arm,drive,verticalSlides,intakePos2,4,0),
                                    humanIntake(arm,verticalSlides),
                                    toIntake2(arm,drive,verticalSlides,preScorePos2),
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                    toScore3(arm,drive,verticalSlides,intakePos2,8,0),
                                    toIntake2(arm,drive,verticalSlides,preScorePos3),
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                    toScore4(arm,drive,verticalSlides,intakePos2,8,0)
                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
