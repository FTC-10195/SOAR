package org.firstinspires.ftc.teamcode.Autonomous;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ActionTelemtry;
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
    Vector2d cameraVec = new Vector2d(sampleVec.x,sampleVec.y);
    double cameraRot = 0;
    Pose2d cameraPos = new Pose2d(sampleVec,Math.toRadians(sampleRot));

    //Sample1

    public Action scouting(){
        return (
                new SequentialAction(
                        arm.shoulderAction(Arm.Shoulder.FORWARDS),
                        arm.wristAction(Arm.Wrist.FULL_DOWNWARDS),
                        arm.extendoAction(Arm.Extendo.EXTENDED),
                        arm.intakeAction(Arm.Intake.INTAKE),
                        verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN)
                )
        );
    }
    public Action intake(){
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
    private Action toSample1(Pose2d pos){
        return
                new SequentialAction(
                       scouting(),
                        new SleepAction(6),
                        webcam.snapshotAction(Arm.TeamColor.NONE),
                        new SleepAction(3),
                        webcam.setDrive(drive,pos),
                    //    new SleepAction(1),
                     //   webcam.snapshotAction(Arm.TeamColor.NONE),
                     //   webcam.updateAction(telemetry,arm),
                        new SleepAction(1),
                        webcam.snapshotAction(Arm.TeamColor.NONE),
                        new SleepAction(1),
                        webcam.setArm(arm,true),
                        intake()
                );
    }
    PinpointDrive drive;
    Arm arm;
    Webcam webcam;
    VerticalSlides verticalSlides;
    ActionTelemtry actionTelemtry;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        actionTelemtry = new ActionTelemtry();
        drive = new PinpointDrive(hardwareMap, beginPose);
        webcam = new Webcam();
        webcam.initiate(hardwareMap, Arm.TeamColor.RED,telemetry);
        verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        arm = new Arm();
        arm.initiate(hardwareMap);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // Main Auto functions
                            new SequentialAction(
                                   toSample1(beginPose)
                            ),
                            verticalSlides.updateAction(),
                            arm.updateAction(telemetry, Arm.TeamColor.NONE),
                            webcam.updateAction(telemetry),
                            actionTelemtry.telemetryAddAction(telemetry,webcam.getSampleRotation()),
                            actionTelemtry.telemetryAddCameraVec(telemetry,cameraVec),
                            actionTelemtry.telemetryAction(telemetry)
                    )
            );
        }
    }
}
