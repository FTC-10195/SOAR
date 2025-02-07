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
    Pose2d beginPose = new Pose2d(0, 0 + BARRIER_OFFSET, Math.toRadians(0)); //Robot starts facing forwards towards the submersible
    //Pre-score
    Vector2d preScoreVec = new Vector2d((TILE_SIZE - ROBOT_LENGTH/2),0); //In between row 1 and row 2
    double preScoreRot = 0;
    Pose2d preScorePos = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
    Vector2d scoreVec = new Vector2d(preScoreVec.x + 11,0); //drives forward enough to score
    double scoreRot = 0;
    Pose2d scorePos = new Pose2d(scoreVec, Math.toRadians(scoreRot));
    Vector2d prePushVec1 = new Vector2d(preScoreVec.x,-TILE_SIZE - 7); //drives left enough to avoid hitting the submersible
    double prePushRot1 = 90;
    Pose2d prePushPos1 = new Pose2d(prePushVec1, Math.toRadians(prePushRot1));
    Vector2d prePushVec2 = new Vector2d(preScoreVec.x + (TILE_SIZE + (TILE_SIZE/2)), prePushVec1.y); //drives forward enough, ex 36 inches, to push the samples
    double prePushRot2 = 90;
    Pose2d prePushPos2 = new Pose2d(prePushVec2, Math.toRadians(prePushRot2));
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
                                .strafeToLinearHeading(preScoreVec, Math.toRadians(preScoreRot))
                                .build()
                );
    }
    private Action toScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(scoreVec, Math.toRadians(scoreRot))
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
                                .strafeToLinearHeading(prePushVec1, Math.toRadians(prePushRot1))
                                .build(),
                        drive.actionBuilder(prePushPos1)
                                .strafeToLinearHeading(prePushVec2, Math.toRadians(prePushRot2))
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
                                   toPreScore(arm,drive,verticalSlides,beginPose),
                                    new SleepAction(.8), //Wait a little to reduce wobble
                                    toScore(arm,drive,verticalSlides,preScorePos),
                                    toPreScore(arm,drive,verticalSlides,scorePos), //Drive backwards
                                    humanIntake(arm,verticalSlides),
                                    toPrePush(arm,drive,verticalSlides,preScorePos)

                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
