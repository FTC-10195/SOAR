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
    double OFFSET = -2.5;
    //Positions
    Pose2d beginPose = new Pose2d(0, 0 + BARRIER_OFFSET, Math.toRadians(0));
    //Pre-score
    Vector2d preScoreVec = new Vector2d(-(ROBOT_LENGTH/2) + OFFSET , (TILE_SIZE - (ROBOT_WIDTH/2)));
    double preScoreRot = 45;
    Pose2d preScorePos = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
    //Score pos, will be used a lot
    Vector2d scoreVec = new Vector2d(-17, -3);
    double scorePosRot = 45;
    Pose2d scorePos = new Pose2d(scoreVec,Math.toRadians(scorePosRot));
    //Sample1
    Vector2d sample1Vec =  new Vector2d(10 - (ROBOT_LENGTH/2) + OFFSET +1 , (TILE_SIZE - ROBOT_WIDTH));
    double sample1Rot = 90;
    Pose2d sample1Pos = new Pose2d(sample1Vec,Math.toRadians(sample1Rot));
    Vector2d sample2Vec =  new Vector2d(-.5 - (ROBOT_LENGTH/2) + OFFSET, (TILE_SIZE - ROBOT_WIDTH));
    double sample2Rot = 90;
    Pose2d sample2Pos = new Pose2d(sample2Vec,Math.toRadians(sample2Rot));

    Vector2d sample3Vec =  new Vector2d(preScoreVec.x + (TILE_SIZE/2) +OFFSET, preScoreVec.y + 3);
    double sample3Rot = 130;
    Pose2d sample3Pos = new Pose2d(sample3Vec,Math.toRadians(sample3Rot));
    Vector2d parkVec1 =  new Vector2d(preScoreVec.x, preScoreVec.y + 37);
    double parkRot1 = 0;
    Pose2d parkPos1 = new Pose2d(parkVec1,Math.toRadians(parkRot1));
    Vector2d parkVec2 =  new Vector2d(parkVec1.x +21, parkVec1.y);
    double parkRot2 = 0;
    Pose2d parkPos2 = new Pose2d(parkVec2,Math.toRadians(parkRot2));
    public Action park(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.UPWARDS),
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN),
                        arm.clawRotationAction(Arm.ClawRotation.Horz1),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        arm.wristAction(Arm.Wrist.FORWARD),
                        arm.extendoAction(Arm.Extendo.RETRACTED)
                )
        );
    }
    public Action bucket(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                    arm.shoulderAction(Arm.Shoulder.BUCKET),
                    arm.clawRotationAction(Arm.ClawRotation.Horz1),
                    arm.intakeAction(Arm.Intake.CLOSE),
                    arm.wristAction(Arm.Wrist.FORWARD),
                    arm.extendoAction(Arm.Extendo.RETRACTED),
                        new SleepAction(.5), //Rotating mechanisms AND moving the slides makes the robot move strangely
                    verticalSlides.slideAction(VerticalSlides.SlidePositions.BUCKET)
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
                        new SleepAction(.3), //Gives time for the extendo to extend
                        arm.shoulderAction(Arm.Shoulder.DOWNWARDS),
                        new SleepAction(.5),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        new SleepAction(.2)
                )
        );
    }
    private Action toPreScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        bucket(arm,verticalSlides),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(preScoreVec, Math.toRadians(preScoreRot))
                                .build()
                );
    }
    private Action toScore(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(scoreVec, Math.toRadians(scorePosRot))
                                .build(),
                        new SleepAction(.6), //Wait a little longer to minimize wobble
                        arm.intakeAction(Arm.Intake.DEPOSIT),
                        new SleepAction(.2)

                );
    }
    private Action toSample1(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                       scouting(arm,verticalSlides),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(sample1Vec, Math.toRadians(sample1Rot))
                                .build(),
                        intake(arm,verticalSlides)
                );
    }
    private Action toSample2(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        scouting(arm,verticalSlides),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(sample2Vec, Math.toRadians(sample2Rot))
                                .build(),
                        intake(arm,verticalSlides)
                );
    }
    private Action toSample3(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        scouting(arm,verticalSlides),
                        arm.clawRotationAction(Arm.ClawRotation.Diag2),
                        drive.actionBuilder(pos)
                                .strafeToLinearHeading(sample3Vec, Math.toRadians(sample3Rot))
                                .build(),
                        intake(arm,verticalSlides)
                );
    }
    private Action toPark(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos){
        return
                new SequentialAction(
                        park(arm,verticalSlides),
                        drive.actionBuilder(pos)
                                .setTangent(90)
                                .splineToLinearHeading(parkPos1,Math.toRadians(180))
                                .setTangent(180)
                                .splineToConstantHeading(parkVec2, Math.toRadians(180))
                                .build(),
                        arm.shoulderAction(Arm.Shoulder.FORWARDS)
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
                                    bucket(arm,verticalSlides),
                                    new SleepAction(.3),
                                    toScore(arm,drive,verticalSlides,sample1Pos),
                                    toSample2(arm,drive,verticalSlides,scorePos),
                                    bucket(arm,verticalSlides),
                                    new SleepAction(.3),
                                    toScore(arm,drive,verticalSlides,preScorePos),
                                    toSample3(arm,drive,verticalSlides,scorePos),
                                    bucket(arm,verticalSlides),
                                    new SleepAction(.3),
                                    toScore(arm,drive,verticalSlides,preScorePos),
                                    toPark(arm,drive,verticalSlides,scorePos)
                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
