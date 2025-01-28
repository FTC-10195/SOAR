package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous
public class OdoBucket extends LinearOpMode {
    //MEASUREMENTS
    double ROBOT_LENGTH = 17.1;
    double ROBOT_WIDTH = 16.38;
    double TILE_SIZE = 24;
    double BARRIER_OFFSET = 0; //Probably not necessary
    double EXTENDO_REACH = 2; //Used as an offset
    //Positions
    Pose2d beginPose = new Pose2d(0, 0 + BARRIER_OFFSET, Math.toRadians(0));
    //Pre-score
    Vector2d preScoreVec = new Vector2d(-(TILE_SIZE/2) , (TILE_SIZE - ROBOT_WIDTH));
    double preScoreRot = 45;
    Pose2d preScorePos = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
    //Score pos, will be used a lot
    Vector2d scoreVec = new Vector2d(-13, 2);
    double scorePosRot = 45;
    Pose2d scorePos = new Pose2d(scoreVec,Math.toRadians(scorePosRot));
    //Sample1
    Vector2d sample1Vec =  new Vector2d(-.5 , (TILE_SIZE - ROBOT_WIDTH-EXTENDO_REACH));
    double sample1Rot = 90;
    Pose2d sample1Pos = new Pose2d(sample1Vec,Math.toRadians(sample1Rot));
    Vector2d sample2Vec =  new Vector2d(-1.5 - (TILE_SIZE/4) , (TILE_SIZE - ROBOT_WIDTH-EXTENDO_REACH));
    double sample2Rot = 90;
    Pose2d sample2Pos = new Pose2d(sample2Vec,Math.toRadians(sample2Rot));
    private Action toPreScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.BACKWARDS),
                        arm.clawRotationAction(Arm.ClawRotation.Horz1),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        arm.wristAction(Arm.Wrist.FORWARD),
                        arm.extendoAction(Arm.Extendo.RETRACTED),
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.BUCKET),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(preScoreVec, Math.toRadians(preScoreRot))
                                .build()
                );
    }
    private Action toScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.BACKWARDS),
                        arm.wristAction(Arm.Wrist.FORWARD),
                        arm.extendoAction(Arm.Extendo.RETRACTED),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        new SleepAction(.5),//Rotating mechanisms AND moving the slides makes the robot move strangely
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.BUCKET),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(scoreVec, Math.toRadians(scorePosRot))
                                .build(),
                        new SleepAction(.5),
                        arm.intakeAction(Arm.Intake.OPEN),
                        new SleepAction(.5)

                );
    }
    private Action toSample1(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.FORWARDS),
                        arm.wristAction(Arm.Wrist.DOWNWARDS),
                        arm.extendoAction(Arm.Extendo.RETRACTED),
                        arm.intakeAction(Arm.Intake.OPEN),
                        new SleepAction(.5), //Rotating mechanisms AND moving the slides makes the robot move strangely
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(sample1Vec, Math.toRadians(sample1Rot))
                                .build(),
                        arm.extendoAction(Arm.Extendo.EXTENDED),
                        new SleepAction(.5), //Gives time for the extendo to extend
                        arm.shoulderAction(Arm.Shoulder.DOWNWARDS),
                        new SleepAction(.5),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        new SleepAction(.5)
                );
    }
    private Action toSample2(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.FORWARDS),
                        arm.wristAction(Arm.Wrist.DOWNWARDS),
                        arm.extendoAction(Arm.Extendo.RETRACTED),
                        arm.intakeAction(Arm.Intake.OPEN),
                        new SleepAction(.5), //Rotating mechanisms AND moving the slides makes the robot move strangely
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(sample2Vec, Math.toRadians(sample2Rot))
                                .build(),
                        arm.extendoAction(Arm.Extendo.EXTENDED),
                        new SleepAction(.5), //Gives time for the extendo to extend
                        arm.shoulderAction(Arm.Shoulder.DOWNWARDS),
                        new SleepAction(.5),
                        arm.intakeAction(Arm.Intake.CLOSE),
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
                                    toScore(arm,drive,verticalSlides,preScorePos),
                                    toSample1(arm,drive,verticalSlides,scorePos),
                                    toPreScore(arm,drive,verticalSlides,sample1Pos),
                                    toScore(arm,drive,verticalSlides,preScorePos),
                                    toSample2(arm,drive,verticalSlides,scorePos)

                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
