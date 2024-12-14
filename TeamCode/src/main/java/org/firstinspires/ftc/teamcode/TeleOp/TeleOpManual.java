package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@TeleOp
public class TeleOpManual extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
            driveTrain.run(gamepad1.left_stick_x * 1.1, -gamepad1.left_stick_y, -gamepad1.right_stick_x,telemetry);
            verticalSlides.reset(gamepad1.options);


            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                if (arm.shoulderState == Arm.Shoulder.DOWN) {
                    arm.shoulder(Arm.Shoulder.RESTING);
                } else if (arm.shoulderState == Arm.Shoulder.RESTING) {
                    arm.shoulder(Arm.Shoulder.RESTING);
                }
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (arm.extendoState == Arm.Extendo.EXTENDED) {
                    arm.extendo(Arm.Extendo.RETRACTED);
                } else if (arm.extendoState == Arm.Extendo.RETRACTED) {
                    arm.extendo(Arm.Extendo.EXTENDED);
                }
            }
            if (currentGamepad1.cross && !previousGamepad1.cross) {
                if (arm.intakeState == Arm.Intake.OUTTAKING) {
                    arm.intakeState = Arm.Intake.STOPPED;
                } else {
                    arm.intakeState = Arm.Intake.OUTTAKING;
                }
            }
        }
        if (currentGamepad1.triangle && !previousGamepad1.triangle) {
            if (arm.intakeState == Arm.Intake.INTAKING) {
                arm.intakeState = Arm.Intake.STOPPED;
            } else {
                arm.intakeState = Arm.Intake.INTAKING;
            }
        }
        if (currentGamepad1.square && !previousGamepad1.square) {
            if (arm.wristState == Arm.Wrist.FORWARD) {
                arm.wristState = Arm.Wrist.SIDEWAYS;
            } else {
                arm.wristState = Arm.Wrist.FORWARD;
            }
        }
        verticalSlides.manual((gamepad1.left_trigger - gamepad1.right_trigger) / 2, false, telemetry);
        arm.update(telemetry,"");
        telemetry.update();
    }
}
