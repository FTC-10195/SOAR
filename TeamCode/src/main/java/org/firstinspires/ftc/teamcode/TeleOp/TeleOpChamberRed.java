package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@TeleOp
public class TeleOpChamberRed extends LinearOpMode {
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
        Arm.TeamColor teamColor = Arm.TeamColor.RED;
        StateMachine.Mode mode = StateMachine.Mode.CHAMBER;
        StateMachine.States state = StateMachine.States.RESTING;
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            driveTrain.run(gamepad1.left_stick_x * 1.1, -gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);
            verticalSlides.reset(gamepad1.options);
            boolean RT = gamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger < 0.1;
            boolean LT = gamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger < 0.1;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;
            boolean SwitchMode = gamepad1.circle && !previousGamepad1.circle;
            boolean SwitchColor = gamepad1.cross && !previousGamepad1.cross;
            double rumblePower = 0;
            int rumbleTime = 0;
            int rumbleBlips = 0;
            if (state == StateMachine.States.SAMPLE_LOADED) {
                rumblePower = 1;
                rumbleTime = 1000;
                rumbleBlips = 10;
                telemetry.addData("rumbing",true);
            }
            gamepad1.rumble(rumblePower, rumblePower, rumbleTime);
            gamepad1.rumbleBlips(rumbleBlips);
            switch (state) {
                case RESTING:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.UPWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case SCOUTING:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case SAMPLE_INTAKE:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.DOWNWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    arm.intake(Arm.Intake.INTAKING);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case BUCKET:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                    break;
                case BUCKET_DEPOSIT:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.OUTTAKING);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                    break;
                case CHAMBER:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.UPWARDS);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                    break;
                case CHAMBER_DEPOSIT:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.UPWARDS);
                    arm.intake(Arm.Intake.OUTTAKING);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                    break;
                case CHAMBER_READY_TO_FIRE:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.STOPPED);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case CHAMBER_HUMAN_DEPOSIT:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.SHOOTING);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case CHAMBER_HUMAN_INTAKE:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.INTAKING);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
            }
            state = stateMachine.updateState(state, mode, RT, LT, RB, LB,arm.isGrabbed(), telemetry);
            mode =  stateMachine.switchMode(mode,SwitchMode);
            teamColor = arm.switchColor(teamColor, SwitchColor);
            arm.update(telemetry, teamColor);
            verticalSlides.update(telemetry);
            telemetry.addData("CurrentState", state);
            telemetry.update();
        }
    }
}
