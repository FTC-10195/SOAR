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
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@Autonomous
public class OdoWebcamTest extends LinearOpMode {
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
    //Score pos, will be used a lot
    Vector2d sampleVec = new Vector2d(1, 0);
    double sampleRot = 0;
    Pose2d samplePos = new Pose2d(sampleVec,Math.toRadians(sampleRot));
    //Sample1

    public Action scouting(Arm arm, VerticalSlides verticalSlides){
        return (
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.FORWARDS),
                        arm.wristAction(Arm.Wrist.FULL_DOWNWARDS),
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
                        new SleepAction(.4),
                        arm.intakeAction(Arm.Intake.CLOSE),
                        new SleepAction(.3)
                )
        );
    }
    private Action toSample1(Arm arm, PinpointDrive drive, VerticalSlides verticalSlides,Pose2d pos,Webcam webcam){

        return
                new SequentialAction(
                       scouting(arm,verticalSlides),
                        drive.actionBuilder(pos)
                                .splineToConstantHeading(sampleVec, Math.toRadians(sampleRot)) // Where intaking starts
                                .build(),
                        webcam.snapshotAction(telemetry, Arm.TeamColor.NONE),
                        drive.actionBuilder(pos)
                                .splineToConstantHeading(new Vector2d(sampleVec.x + webcam.getTargetVectorDistance().y,sampleVec.x + webcam.getTargetVectorDistance().y), Math.toRadians(sampleRot)) // Where intaking starts
                                .build(),
                        arm.clawRotationAction(webcam.sampleRotation),
                        intake(arm,verticalSlides)
                );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Arm arm = new Arm();
        Webcam webcam = new Webcam();
        webcam.initiate(hardwareMap,telemetry);
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // Main Auto functions
                            new SequentialAction(
                                   toSample1(arm,drive,verticalSlides,beginPose,webcam)
                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE),
                            webcam.updateAction(telemetry,Arm.TeamColor.NONE)
                    )
            );
        }
    }
}
