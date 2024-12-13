package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@TeleOp
public class TeleOpStateMachineOld extends LinearOpMode {
    public enum States {
        RESTING,
        INTAKING,
        INTAKE_STOP,
        BUCKET,
        BUCKET_DEPOSIT,
        CHAMBER,
        CHAMBER_DEPOSIT

    }

    States state = States.RESTING;

    @Override
    public void runOpMode() throws InterruptedException {
        StateMachine stateMachine = new StateMachine();
        Arm arm = new Arm();
        DriveTrain driveTrain = new DriveTrain();
        VerticalSlides verticalSlides = new VerticalSlides();
        driveTrain.initiate(hardwareMap);
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            driveTrain.run(gamepad1.left_stick_x * 1.1, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            verticalSlides.reset(gamepad1.options);

            switch (state) {
                case RESTING:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.RESTING);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case INTAKING:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.DOWN);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.INTAKING);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case INTAKE_STOP:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.DOWN);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case BUCKET:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.RESTING);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                    break;
                case BUCKET_DEPOSIT:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.RESTING);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.OUTTAKING);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                    break;
                case CHAMBER:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.RESTING);
                    arm.wrist(Arm.Wrist.SIDEWAYS);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                    break;
                case CHAMBER_DEPOSIT:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.RESTING);
                    arm.wrist(Arm.Wrist.SIDEWAYS);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
            }

            if (currentGamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger < 0.1) {
                if (state == States.RESTING) {
                    state = States.CHAMBER;
                } else if (state == States.CHAMBER) {
                    state = States.CHAMBER_DEPOSIT;
                } else if (state == States.CHAMBER_DEPOSIT) {
                    state = States.RESTING;
                }
            }
            if (currentGamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger < 0.1) {
                if (state == States.RESTING) {
                    state = States.BUCKET;
                } else if (state == States.BUCKET) {
                    state = States.BUCKET_DEPOSIT;
                } else if (state == States.BUCKET_DEPOSIT) {
                    state = States.RESTING;
                }
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                if (state == States.RESTING) {
                    state = States.INTAKING;
                } else if (state == States.INTAKING) {
                    state = States.INTAKE_STOP;
                } else if (state == States.INTAKE_STOP) {
                    state = States.INTAKING;
                }
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                state = States.RESTING;
            }
            //  stateMachine.update(state);
            if (currentGamepad1.cross) {
                arm.intake(Arm.Intake.OUTTAKING);
            }
            arm.update();
            verticalSlides.setSlidePosition(stateMachine.slidePosition);
            verticalSlides.update();
            telemetry.addData("CurrentState", state);
            telemetry.update();
        }
    }
}
