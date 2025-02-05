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
    Vector2d preScoreVec = new Vector2d(0,(TILE_SIZE - ROBOT_LENGTH/2)); //In between row 1 and row 2
    double preScoreRot = 0;
    Pose2d preScorePos = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
    Vector2d scoreVec = new Vector2d(0,(TILE_SIZE + TILE_SIZE - ROBOT_LENGTH/2)); //At submersible
    double scoreRot = 0;
    Pose2d scorePos = new Pose2d(scoreVec, Math.toRadians(scoreRot));
    double chamberNumber = 0; //Used as an offset for each new chamber score
    public Action chamber(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                    arm.shoulderAction(Arm.Shoulder.FORWARDS),
                    arm.clawRotationAction(Arm.ClawRotation.Horz1),
                    arm.intakeAction(Arm.Intake.CLOSE),
                    arm.wristAction(Arm.Wrist.DOWNWARDS),
                    arm.extendoAction(Arm.Extendo.RETRACTED),
                    verticalSlides.slideAction(VerticalSlides.SlidePositions.CHAMBER),
                        new SleepAction(.5),
                        arm.extendoAction(Arm.Extendo.CHAMBER) //Wait slightly before extending because rotating rotation mech and extending extendo doesn't work well
                )
        );
    }
    public Action scouting(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.FORWARDS),
                        arm.wristAction(Arm.Wrist.DOWNWARDS),
                        arm.extendoAction(Arm.Extendo.RETRACTED),
                        arm.intakeAction(Arm.Intake.INTAKE),
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN)
                )
        );
    }
    public Action intake(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                        arm.extendoAction(Arm.Extendo.EXTENDED),
                        new SleepAction(.5), //Gives time for the extendo to extend
                        arm.shoulderAction(Arm.Shoulder.DOWNWARDS),
                        new SleepAction(.5),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        new SleepAction(.5)
                )
        );
    }
    private Action toPreScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        chamber(arm,verticalSlides),
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
                        new SleepAction(.5)

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
                                   toPreScore(arm,drive,verticalSlides,beginPose),
                                    toScore(arm,drive,verticalSlides,preScorePos)
                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
