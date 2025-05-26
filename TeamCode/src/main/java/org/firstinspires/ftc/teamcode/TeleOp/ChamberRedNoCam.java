package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Ascent;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
//import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@TeleOp
public class ChamberRedNoCam extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
      //  Webcam webcam = new Webcam();
      //  webcam.initiate(hardwareMap,telemetry);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        waitForStart();
        if (isStopRequested()) return;
        StateMachine stateMachine = new StateMachine();
        Arm arm = new Arm();
        DriveTrain driveTrain = new DriveTrain();
        VerticalSlides verticalSlides = new VerticalSlides();
        driveTrain.initiate(hardwareMap);
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        arm.shoulder(Arm.Shoulder.FORWARDS);
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
        boolean ascentA = false;
        boolean ascentSquare = false;
        Ascent ascent = new Ascent();
        ascent.initiate(hardwareMap);
        while (opModeIsActive()) {
       //     webcam.rotate(arm.getClawRotation(),telemetry);
            drive.updatePoseEstimate();
            ascent.update(ascentSquare, telemetry);
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            verticalSlides.reset(gamepad1.options);
            boolean RT = gamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger < 0.1;
            boolean LT = gamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger < 0.1;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;
            boolean switchMode = gamepad1.circle && !previousGamepad1.circle;
            boolean switchColor = gamepad1.cross && !previousGamepad1.cross;
            mode = stateMachine.switchMode(mode,switchMode);
            state = stateMachine.setState(state,mode, RT, LT, RB, LB, telemetry);
            switch (state) {
                case RESTING:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.UPWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                    break;
                case SCOUTING:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.FULL_DOWNWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case SAMPLE_INTAKE:
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    break;
                case BUCKET:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.BUCKET);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                    break;
                case CHAMBER:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                    break;
                case CHAMBER_DEPOSIT:
                    arm.extendo(Arm.Extendo.CHAMBER);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                    break;
                case CHAMBER_PRE_DEPOSIT:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    break;
                case CHAMBER_HUMAN_INTAKE:
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.CHAMBER_INTAKE);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                    break;
            }
         arm.intake(stateMachine.clawState);
            if (gamepad1.triangle && ascentA == false){
                ascentA = true;
                switch (ascent.climbPosition){
                    case DOWN:
                        ascent.setClimb(Ascent.ClimbPositions.PLACE);
                        break;
                    case PLACE:
                        ascent.setClimb(Ascent.ClimbPositions.MAX);
                        break;
                    case MAX:
                        ascent.setClimb(Ascent.ClimbPositions.PLACE);
                        break;
                }
            }else if (!gamepad1.triangle){
                ascentA = false;
            }
            if (gamepad1.square){
                ascentSquare = true;
            }
            if (gamepad1.dpad_left){
                arm.clawRotate(Arm.ClawRotation.Diag1);
            }
            if (gamepad1.dpad_right){
                arm.clawRotate(Arm.ClawRotation.Diag2);
            }
            if (gamepad1.dpad_up){
                arm.clawRotate(Arm.ClawRotation.Horz1);
            }
            if (gamepad1.dpad_down){
                arm.clawRotate(Arm.ClawRotation.Vert);
            }

            ascent.reset(gamepad1.options);
            telemetry.addData("rt2current",gamepad2.right_trigger);
            telemetry.addData("rt2prev",previousGamepad2.right_trigger);
            teamColor = arm.switchColor(teamColor,switchColor);
            arm.update(telemetry, teamColor);
            verticalSlides.update();
            telemetry.addData("CurrentState", state);
            telemetry.update();
        }
    }
}
