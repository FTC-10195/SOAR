package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous
public class OdoBucket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, -63.75, Math.toRadians(180));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        Arm arm = new Arm();
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // Main Auto functions
                            new SequentialAction(
                                    arm.intakeAction(Arm.Intake.CLOSE),
                                    new ParallelAction(
                                            mecanumDrive.actionBuilder(beginPose)
                                                    .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                                                    .build(),
                                            arm.bucketAction(),
                                            verticalSlides.slideAction(VerticalSlides.SlidePositions.BUCKET)
                                    ),
                                    mecanumDrive.actionBuilder(beginPose)
                                            .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(225))
                                            .build(),
                                    arm.intakeAction(Arm.Intake.OPEN),
                                    new SleepAction(.5),
                                    mecanumDrive.actionBuilder(beginPose)
                                            .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                                            .build(),
                                    verticalSlides.slideAction(VerticalSlides.SlidePositions.DOWN)
                            ),
                            verticalSlides.updateAction()
                    )
            );
        }
    }
}
