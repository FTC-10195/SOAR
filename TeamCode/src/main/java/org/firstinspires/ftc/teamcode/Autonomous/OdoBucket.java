package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.config.Config;
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
    @Override
    public void runOpMode() throws InterruptedException {
        double robotLength = 17.1;
        double robotWidth = 16.38;
        double tileSize = 24;
        double scoreOffset = 6;
        double barrierOffset = 0;
        double extendoReach = 2.5;
        double bucketIntakeAngle = 45;
        double sampleIntakeAngle1 = 90;
        Pose2d beginPose = new Pose2d(0, 0 + barrierOffset, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);        Arm arm = new Arm();
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        waitForStart();
        Vector2d preScorePos = new Vector2d(0 , (tileSize - robotWidth));
        Vector2d scorePos = new Vector2d(-13, 5);
        Vector2d sample2Pos = new Vector2d(-.5 , (tileSize - robotWidth-extendoReach));
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // Main Auto functions
                            new SequentialAction(
                                    arm.intakeAction(Arm.Intake.CLOSE),
                                    new ParallelAction(
                                            drive.actionBuilder(beginPose)
                                                    .strafeToLinearHeading(preScorePos, Math.toRadians(bucketIntakeAngle))
                                                    .build(),
                                            arm.shoulderAction(Arm.Shoulder.BACKWARDS),
                                            arm.wristAction(Arm.Wrist.FORWARD),
                                            arm.extendoAction(Arm.Extendo.RETRACTED),
                                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN)
                                    ),
                                    new SleepAction(.5),
                                    drive.actionBuilder(new Pose2d(preScorePos,Math.toRadians(bucketIntakeAngle)))
                                            .strafeToLinearHeading(scorePos, Math.toRadians(bucketIntakeAngle))
                                            .build(),
                                    verticalSlides.slideAction(VerticalSlides.SlidePositions.BUCKET),
                                    new SleepAction(.5),
                                    arm.shoulderAction(Arm.Shoulder.BACKWARDS),
                    arm.extendoAction(Arm.Extendo.RETRACTED),
                    new SleepAction(1.5),
                                    arm.intakeAction(Arm.Intake.OPEN),
                                    arm.shoulderAction(Arm.Shoulder.BACKWARDS),
                                    new SleepAction(.5),
                                   verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN),
                                    arm.shoulderAction(Arm.Shoulder.FORWARDS),
                                    arm.extendoAction(Arm.Extendo.RETRACTED),
                                    arm.wristAction(Arm.Wrist.DOWNWARDS),
                                    new SleepAction(1),
                                    drive.actionBuilder(new Pose2d(scorePos,Math.toRadians(bucketIntakeAngle)))
                                            .strafeToLinearHeading(sample2Pos, Math.toRadians(bucketIntakeAngle))
                                            .turnTo(Math.toRadians(sampleIntakeAngle1))
                                            .build(),
                                    new SleepAction(1),
                                    arm.extendoAction(Arm.Extendo.EXTENDED),
                                    new SleepAction(.5),
                                    arm.shoulderAction(Arm.Shoulder.DOWNWARDS),
                                    new SleepAction(.5),
                                    arm.intakeAction(Arm.Intake.CLOSE),
            new SleepAction(.5),
                    drive.actionBuilder(new Pose2d(preScorePos,Math.toRadians(bucketIntakeAngle)))
                            .strafeToLinearHeading(scorePos, Math.toRadians(bucketIntakeAngle))
                            .build(),
                                    arm.shoulderAction(Arm.Shoulder.BACKWARDS),
                                    arm.wristAction(Arm.Wrist.FORWARD),
                                    arm.extendoAction(Arm.Extendo.RETRACTED),
                                    new SleepAction(.5),
                                    verticalSlides.slideAction(VerticalSlides.SlidePositions.BUCKET),
                                    new SleepAction(2),
                                    arm.intakeAction(Arm.Intake.OPEN),
                                    arm.shoulderAction(Arm.Shoulder.BACKWARDS),
                                    new SleepAction(.5),
                                    verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN),
                                    arm.shoulderAction(Arm.Shoulder.FORWARDS),
                                    arm.extendoAction(Arm.Extendo.RETRACTED),
                                    arm.wristAction(Arm.Wrist.DOWNWARDS)
                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
