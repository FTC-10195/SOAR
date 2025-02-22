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
public class TeleOpBucketRed extends LinearOpMode {
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
        StateMachine.Mode mode = StateMachine.Mode.BUCKET;
        StateMachine.States state = StateMachine.States.RESTING;
        Webcam webcam = new Webcam();
        webcam.initiate(hardwareMap,teamColor,mode,telemetry);
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
            boolean SwitchMode = gamepad1.circle && !previousGamepad1.circle;
            boolean SwitchColor = gamepad1.cross && !previousGamepad1.cross;
            mode = stateMachine.switchMode(mode,SwitchMode);
            state = stateMachine.setState(state,mode, RT, LT, RB, LB, telemetry);
            if (state != StateMachine.States.SAMPLE_INTAKE && arm.isLerpComplete()){
                webcam.updateCurrentDriveStage(Webcam.DRIVE_STAGE.DONE);
            }else if (LB && state == StateMachine.States.SAMPLE_INTAKE){
                webcam.updateCurrentDriveStage(Webcam.DRIVE_STAGE.MOVE_TO_TARGET);
                webcam.updateDriveStartPos(drive.pinpoint.getPositionRR());
                webcam.snapshot();
                arm.intake(Arm.Intake.INTAKE);
            }
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
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                    break;
                case SAMPLE_INTAKE:

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
            ascent.reset(gamepad1.options);
            telemetry.addData("rt2current",gamepad2.right_trigger);
            telemetry.addData("rt2prev",previousGamepad2.right_trigger);
            teamColor = arm.switchColor(teamColor,SwitchColor);
            arm.update(telemetry, teamColor);
            verticalSlides.update();
            webcam.webcamDrive(drive,arm,teamColor,telemetry);
            if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE){
                driveTrain.run(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x,telemetry);
                if (arm.isLerpComplete() && state == StateMachine.States.SAMPLE_INTAKE && System.currentTimeMillis() - arm.shoulderLerpStartTime < 1000){
                    stateMachine.setClawState(Arm.Intake.CLOSE);
                }
            }
            webcam.loop(telemetry);
            telemetry.addData("CurrentState", state);
            telemetry.update();
        }
    }
}
