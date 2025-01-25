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
        StateMachine.Mode mode = StateMachine.Mode.CHAMBER;
        StateMachine.States state = StateMachine.States.RESTING;
        boolean clawRotationRanLeft = false;
        boolean clawRotationRanRight = false;
        boolean clawRan = false;
        boolean clawRB2 = false;
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
            boolean SwitchMode = gamepad1.circle && !previousGamepad1.circle;
            boolean SwitchColor = gamepad1.cross && !previousGamepad1.cross;
            int rumbleCount = 0;
            if (state == StateMachine.States.SAMPLE_LOADED) {
                rumbleCount = 10;
            }
            gamepad1.rumbleBlips(rumbleCount);
            stateMachine.switchMode(mode,SwitchMode);
            state = stateMachine.setState(state,mode, RT, LT, RB, LB, telemetry);
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
                    arm.clawRotate(Arm.ClawRotation.Horz2);
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
            if (gamepad2.left_trigger >=.1 && clawRotationRanLeft == false){
                clawRotationRanLeft = true;
                if (arm.clawRotation == Arm.ClawRotation.Vert){
                    arm.clawRotate(Arm.ClawRotation.Diag1);
                }else if (arm.clawRotation == Arm.ClawRotation.Diag1){
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                }else if (arm.clawRotation == Arm.ClawRotation.Horz1){
                    arm.clawRotate(Arm.ClawRotation.Diag2);
                }else if (arm.clawRotation == Arm.ClawRotation.Diag2){
                    arm.clawRotate(Arm.ClawRotation.Vert);
                }
            }else if (gamepad2.left_trigger < .1){
                clawRotationRanLeft = false;
            }
            if (gamepad2.right_trigger >=.1 && clawRotationRanRight == false){
                clawRotationRanRight = true;
                if (arm.clawRotation == Arm.ClawRotation.Vert){
                    arm.clawRotate(Arm.ClawRotation.Diag2);
                }else if (arm.clawRotation == Arm.ClawRotation.Diag2){
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                }else if (arm.clawRotation == Arm.ClawRotation.Horz1){
                    arm.clawRotate(Arm.ClawRotation.Diag1);
                }else if (arm.clawRotation == Arm.ClawRotation.Diag1){
                    arm.clawRotate(Arm.ClawRotation.Vert);
                }
            }else if (gamepad2.right_trigger < .1){
                clawRotationRanRight = false;
            }
            if (gamepad2.left_bumper && clawRan == false){
                clawRan = true;
                if (arm.intakeState == Arm.Intake.OPEN){
                    arm.intake(Arm.Intake.CLOSE);
                }else if (arm.intakeState == Arm.Intake.CLOSE){
                    arm.intake(Arm.Intake.OPEN);
                }
            }else if (!gamepad2.left_bumper){
                clawRan = false;
            }
            if (gamepad2.right_bumper && clawRB2 == false){
                clawRB2 = true;
                if (arm.clawRotation == Arm.ClawRotation.Horz1){
                    arm.clawRotate(Arm.ClawRotation.Horz2);
                }else if (arm.clawRotation == Arm.ClawRotation.Horz2){
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                }
            }else if (!gamepad2.right_bumper){
                clawRB2 = false;
            }
            telemetry.addData("rt2current",gamepad2.right_trigger);
            telemetry.addData("rt2prev",previousGamepad2.right_trigger);
            teamColor = arm.switchColor(teamColor,SwitchColor);
            arm.update(telemetry, teamColor);
            verticalSlides.update();
            telemetry.addData("CurrentState", state);
            telemetry.update();
        }
    }
}
