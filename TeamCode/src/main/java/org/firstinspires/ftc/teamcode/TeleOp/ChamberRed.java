package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Ascent;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
//import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@TeleOp
public class ChamberRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //  Webcam webcam = new Webcam();
        //  webcam.initiate(hardwareMap,telemetry);
        // PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        Constants constants;
        Follower follower;
        waitForStart();
        if (isStopRequested()) return;
        StateMachine stateMachine = new StateMachine();
        boolean webcamActive = true;
        Arm arm = new Arm();
        DriveTrain driveTrain = new DriveTrain();
        constants = new Constants();
        constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        VerticalSlides verticalSlides = new VerticalSlides();
        driveTrain.initiate(hardwareMap);
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        arm.shoulder(Arm.Shoulder.FORWARDS);
        TeamColor teamColor = new TeamColor(TeamColor.Color.RED);
        teamColor.initiate(hardwareMap);
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        StateMachine.Mode mode = StateMachine.Mode.CHAMBER;
        StateMachine.States state = StateMachine.States.RESTING;
        Webcam webcam = new Webcam();
        webcam.initiate(hardwareMap, teamColor.getColor(), mode, telemetry);
        Pose lockPoint = new Pose(0,0,Math.toRadians(0));
        boolean holdPID = false;
        boolean runHeadlights = false;
        boolean clawRotationRanLeft = false;
        boolean clawRotationRanRight = false;
        boolean clawRan = false;
        boolean clawRB2 = false;
        boolean ascentA = false;
        boolean ascentSquare = false;
        Ascent ascent = new Ascent();
        ascent.initiate(hardwareMap);
        long extendoResetTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            //     webcam.rotate(arm.getClawRotation(),telemetry);
            ascent.update(ascentSquare, telemetry);
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            verticalSlides.reset(gamepad1.options || gamepad2.right_bumper);
            if (gamepad2.right_trigger > .1){
                verticalSlides.offset += verticalSlides.offsetGain;
            }
            if (gamepad2.left_trigger > .1){
                verticalSlides.offset -= verticalSlides.offsetGain;
            }
            boolean up2 = gamepad2.dpad_up && !previousGamepad2.dpad_up;
            boolean down2 = gamepad2.dpad_down && !previousGamepad2.dpad_down;
            boolean triangle2 = gamepad2.triangle && !previousGamepad2.triangle;
            boolean cross2 = gamepad2.cross && !previousGamepad2.cross;
            boolean circle2 = gamepad2.circle && !previousGamepad2.circle;
            boolean LB2 = gamepad2.left_bumper && !previousGamepad2.left_bumper;
            if (LB2){
                lockPoint = follower.getPose();
                holdPID = !holdPID;
            }
            if (up2){
                arm.wristOffset += arm.wristOffsetGain;
            }
            if (down2){
                arm.wristOffset -= arm.wristOffsetGain;
            }
            if (triangle2){
                arm.shoulderOffset += arm.shoulderOffsetGain;
            }
            if (cross2){
                arm.shoulderOffset -= arm.shoulderOffsetGain;
            }
            if (circle2){
                verticalSlides.secondaryBucket = !verticalSlides.secondaryBucket;
            }

            boolean RT = gamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger < 0.1;
            boolean LT = gamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger < 0.1;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;
            if ((LB && state == StateMachine.States.SAMPLE_INTAKE) || (LB && state == StateMachine.States.RESTING) || (RT && state == StateMachine.States.CHAMBER_HUMAN_INTAKE)) {
                extendoResetTime = System.currentTimeMillis();
            }
            boolean switchCameraActive = gamepad1.share && !previousGamepad1.share;
            boolean switchMode = gamepad1.circle && !previousGamepad1.circle;
            boolean switchColor = gamepad1.cross && !previousGamepad1.cross;

            Arm.ClawRotation clawRotOveride = Arm.ClawRotation.Horz1;

            mode = stateMachine.switchMode(mode, switchMode);
            state = stateMachine.setState(state, mode, RT, LT, RB, LB, telemetry);
            teamColor.status(telemetry);
            if (switchCameraActive){
                webcamActive = !webcamActive;
            }
            if (!webcamActive){
                if (gamepad1.dpad_up){
                    clawRotOveride = Arm.ClawRotation.Horz1;
                }
                if (gamepad1.dpad_left){
                    clawRotOveride = Arm.ClawRotation.LEFTDIAG;
                }
                if (gamepad1.dpad_right){
                    clawRotOveride = Arm.ClawRotation.RIGHTDIAG;
                }
                if (gamepad1.dpad_down){
                    clawRotOveride = Arm.ClawRotation.Vert;
                }
            }
            if (state != StateMachine.States.SAMPLE_INTAKE && arm.isLerpComplete()) {
                webcam.setDriveStage(Webcam.DRIVE_STAGE.DONE);
            } else if (LB && state == StateMachine.States.SAMPLE_INTAKE) {
                arm.intake(Arm.Intake.INTAKE);
                webcam.setDriveStage(Webcam.DRIVE_STAGE.DRIVE);
                if (!webcamActive) {
                    webcam.setClawRotation(clawRotOveride);
                    arm.clawRotate(clawRotOveride);
                    webcam.setDriveStage(Webcam.DRIVE_STAGE.DROP);
                    arm.shoulderLerpStartTime = System.currentTimeMillis();
                    arm.shoulder(Arm.Shoulder.DOWNWARDS);
                }
            }
            if (switchMode) {
                webcam.setColorLocatorMode(mode, false);
            }
            if (switchColor) {
                teamColor.flipColor();
                webcam.setColorLocatorTeam(teamColor.getColor(), false);
            }
            //Save processing power idk
            if (state == StateMachine.States.SCOUTING || state == StateMachine.States.SAMPLE_INTAKE) {
                webcam.setLiveView(true);
            } else {
                webcam.setLiveView(false);
            }
            switch (state) {
                case RESTING:
                    runHeadlights = false;
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.shoulder(Arm.Shoulder.UPWARDS);
                    arm.wrist(Arm.Wrist.FORWARD);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                    webcam.setClawRotation(Arm.ClawRotation.Horz1);
                    break;
                case SCOUTING:
                    runHeadlights = true;
                    if (System.currentTimeMillis() - extendoResetTime > 400) {
                        arm.extendo(Arm.Extendo.EXTENDED);
                    } else {
                        arm.extendo(Arm.Extendo.RETRACTED);
                    }
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.FULL_DOWNWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                    webcam.setClawRotation(Arm.ClawRotation.Horz1);
                    break;
                case SAMPLE_INTAKE:
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    break;
                case BUCKET:
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.shoulder(Arm.Shoulder.BUCKET);
                    if (verticalSlides.secondaryBucket){
                        arm.clawRotate(Arm.ClawRotation.Horz1);
                        arm.wrist(Arm.Wrist.FORWARD);
                    }else{
                        arm.clawRotate(Arm.ClawRotation.Vert);
                        arm.wrist(Arm.Wrist.UPWARDS);
                    }
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                    break;
                case CHAMBER:
                    if (System.currentTimeMillis() - extendoResetTime > 400) {
                        arm.extendo(Arm.Extendo.CHAMBER);
                    } else {
                        arm.extendo(Arm.Extendo.RETRACTED);
                    }
                    arm.shoulder(Arm.Shoulder.CHAMBER_SCORE);
                    arm.clawRotate(Arm.ClawRotation.Horz1);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                    break;
                case CHAMBER_DEPOSIT:
                    arm.extendo(Arm.Extendo.CHAMBER);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.FULL_DOWNWARDS);
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
            if (gamepad1.triangle && ascentA == false) {
                ascentA = true;
                switch (ascent.climbPosition) {
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
            } else if (!gamepad1.triangle) {
                ascentA = false;
            }
            if (gamepad1.square) {
                ascentSquare = true;
            }
            teamColor.runHeadlights(runHeadlights);
            TelemetryPacket packet = new TelemetryPacket();
            webcam.update(driveTrain, arm, packet);
            ascent.reset(gamepad1.options);
            arm.update(telemetry, teamColor.getColor());
            verticalSlides.status(telemetry);
            verticalSlides.update();

            if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE) {
                driveTrain.run(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DROP) {
                stateMachine.setClawState(webcam.intakeState);
                arm.intake(webcam.intakeState);
                driveTrain.run(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else {
                arm.intake(webcam.intakeState);
                stateMachine.setClawState(webcam.intakeState);
            }
            if (holdPID){
                follower.holdPoint(lockPoint);
                gamepad1.rumble(50);
                gamepad2.rumble(50);
            }
            arm.intake(stateMachine.clawState);
            webcam.status(telemetry);
            webcam.statusFTCDashboard(packet);
            teamColor.update();
            telemetry.addData("CurrentState", state);
            telemetry.update();
            driveTrain.getStatus(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
