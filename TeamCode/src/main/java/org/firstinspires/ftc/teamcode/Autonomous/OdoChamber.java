package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.Action;
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

@Autonomous
public class OdoChamber extends LinearOpMode {
    //MEASUREMENTS
    double ROBOT_LENGTH = 17.1;
    double ROBOT_WIDTH = 16.38;
    double TILE_SIZE = 24;
    double BARRIER_OFFSET = 0; //Probably not necessary
    double EXTENDO_REACH = 2; //Used as an offset
    //Positions
    Vector2d beginVec = new Vector2d ((ROBOT_WIDTH/2),-72+(ROBOT_LENGTH/2));
    double beginRot = 90;
    Pose2d beginPose = new Pose2d(beginVec , Math.toRadians(beginRot)); //Robot starts facing forwards towards the submersible

    //Pre-score
    Vector2d preScoreVec = new Vector2d(beginVec.x,beginVec.y + (TILE_SIZE - ROBOT_LENGTH/2)); //In between row 1 and row 2
    double preScoreRot = 90;
    Pose2d preScorePos = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
    Vector2d scoreVec = new Vector2d(preScoreVec.x, preScoreVec.y + 11); //drives forward enough to score
    double scoreRot = preScoreRot;
    Pose2d scorePos = new Pose2d(scoreVec, Math.toRadians(scoreRot));
    Vector2d prePushVec1 = new Vector2d(preScoreVec.x + TILE_SIZE + 4, preScoreVec.y); //drives left enough to avoid hitting the submersible
    double prePushRot1 = 0;
    Pose2d prePushPos1 = new Pose2d(prePushVec1, Math.toRadians(prePushRot1));
    Vector2d prePushVec2 = new Vector2d(prePushVec1.x, prePushVec1.y+ (TILE_SIZE + (TILE_SIZE/2))); //drives forward enough, ex 36 inches, to push the samples
    double prePushRot2 = 0;
    Pose2d prePushPos2 = new Pose2d(prePushVec2, Math.toRadians(prePushRot2));
    Vector2d prePushVec3 = new Vector2d(prePushVec2.x + (TILE_SIZE/2) - 3, prePushVec2.y); //drives above the first sample
    double prePushRot3 = 0;
    Pose2d prePushPos3 = new Pose2d(prePushVec3, Math.toRadians(prePushRot3));
    Vector2d pushVec1 = new Vector2d(prePushVec3.x , prePushVec3.y - 38); //drives above the first sample
    double pushRot1 = 0;
    Pose2d pushPos1 = new Pose2d(pushVec1, Math.toRadians(pushRot1));

    Vector2d prePushVec4 = new Vector2d(prePushVec2.x + (TILE_SIZE/2) + 7, prePushVec2.y); //drives above the first sample
    double prePushRot4 = 0;
    Pose2d prePushPos4 = new Pose2d(prePushVec4, Math.toRadians(prePushRot4));

    Vector2d pushVec2 = new Vector2d(prePushVec4.x , prePushVec4.y - 38); //drives above the first sample
    double pushRot2 = 0;
    Pose2d pushPos2 = new Pose2d(pushVec2, Math.toRadians(pushRot2));

    Vector2d prePushVec5 = new Vector2d(prePushVec2.x + (TILE_SIZE/2) + 12, prePushVec4.y); //drives above the first sample
    double prePushRot5 = 0;
    Pose2d prePushPos5 = new Pose2d(prePushVec5, Math.toRadians(prePushRot5));
    double chamberNumber = 0; //Used as an offset for each new chamber score
    public Action chamber(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                    arm.shoulderAction(Arm.Shoulder.FORWARDS),
                    arm.clawRotationAction(Arm.ClawRotation.Horz1),
                    arm.intakeAction(Arm.Intake.CLOSE),
                    arm.wristAction(Arm.Wrist.DOWNWARDS),
                    arm.extendoAction(Arm.Extendo.RETRACTED),
                    new SleepAction(.5), //Allow the mechanisms to be in the correct positions before raising slides
                    verticalSlides.slideAction(VerticalSlides.SlidePositions.CHAMBER)
                )
        );
    }
    public Action humanIntake(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                        arm.extendoAction(Arm.Extendo.RETRACTED),
                        arm.shoulderAction(Arm.Shoulder.CHAMBER_INTAKE),
                        arm.wristAction(Arm.Wrist.FORWARD),
                        arm.clawRotationAction(Arm.ClawRotation.Horz1),
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN)
                )
        );
    }
    private Action toPreScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .setTangent(Math.toRadians(beginRot))
                                .splineToConstantHeading(preScoreVec, Math.toRadians(preScoreRot))
                                .build()
                );
    }
    private Action toScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .splineToConstantHeading(scoreVec, Math.toRadians(scoreRot))
                                .build(),
                        new SleepAction(.5),
                        arm.intakeAction(Arm.Intake.DEPOSIT),
                        new SleepAction(.2)

                );
    }
    private Action toPrePush(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .splineToConstantHeading(prePushVec1, Math.toRadians(prePushRot1))
                                .build(),
                        drive.actionBuilder(prePushPos1)
                                .splineToConstantHeading(prePushVec2, Math.toRadians(prePushRot2))
                                .build()

                );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Arm arm = new Arm();
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // Main Auto functions
                            new SequentialAction(
                                    chamber(arm,verticalSlides), //Sets the arm to be ready to chamber
                                   toPreScore(arm,drive,verticalSlides,beginPose)
                               //     new SleepAction(.8), //Wait a little to reduce wobble
                                //    toScore(arm,drive,verticalSlides,preScorePos),
                                //    toPreScore(arm,drive,verticalSlides,scorePos), //Drive backwards
                                //    humanIntake(arm,verticalSlides),
                                 //   toPrePush(arm,drive,verticalSlides,preScorePos)

                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
