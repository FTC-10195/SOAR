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
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Arm.TeamColor teamColor = Arm.TeamColor.RED;
        Arm.Intake clawState = Arm.Intake.CLOSE;
        Arm.ClawRotation clawRotation = Arm.ClawRotation.Vert;
        StateMachine.Mode mode = StateMachine.Mode.CHAMBER;
        StateMachine.States state = StateMachine.States.RESTING;
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            driveTrain.run(gamepad1.left_stick_x * 1.1, -gamepad1.left_stick_y, -gamepad1.right_stick_x, telemetry);
            verticalSlides.reset(gamepad1.options);
            boolean RT = gamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger < 0.1;
            boolean LT = gamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger < 0.1;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;
            boolean RB2 = gamepad2.right_bumper && !previousGamepad2.right_bumper;
            boolean LB2 = gamepad2.left_bumper && !previousGamepad2.left_bumper;
            boolean SwitchMode = gamepad1.circle && !previousGamepad1.circle;
            boolean SwitchColor = gamepad1.cross && !previousGamepad1.cross;
            int rumbleCount = 0;
            if (state == StateMachine.States.SAMPLE_LOADED) {
                rumbleCount = 10;
            }
            gamepad1.rumbleBlips(rumbleCount);
            stateMachine.switchMode(mode,SwitchMode);
            stateMachine.setState(state, mode, RT, LT, RB, LB, SwitchMode, telemetry);
            switch (state) {
                case RESTING:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.UPWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case SCOUTING:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case SAMPLE_INTAKE:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.DOWNWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case BUCKET:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                    break;
                case CHAMBER:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.UPWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                    break;
                case CHAMBER_PRE_DEPOSIT:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BACKWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case CHAMBER_HUMAN_INTAKE:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
            }
            teamColor = arm.switchColor(teamColor,SwitchColor);
            clawState = arm.switchClaw(clawState,LB2);
            clawRotation = arm.switchClawRotation(clawRotation,RB2);
            arm.update(telemetry, teamColor,clawState,clawRotation);
            verticalSlides.update();
            telemetry.addData("CurrentState", state);
            telemetry.update();
        }
    }
}
