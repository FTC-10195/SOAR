package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
//import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@TeleOp
public class TeleOpWebcamDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
      //  Webcam webcam = new Webcam();
      //  webcam.initiate(hardwareMap,telemetry);
        waitForStart();
        if (isStopRequested()) return;

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.initiate(hardwareMap);
        Webcam webcam = new Webcam();
        webcam.initiate(hardwareMap, Arm.TeamColor.RED, StateMachine.Mode.CHAMBER,telemetry);
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Arm arm = new Arm();
        arm.initiate(hardwareMap);
        Arm.TeamColor teamColor = Arm.TeamColor.RED;
        StateMachine.Mode mode = StateMachine.Mode.CHAMBER;
        StateMachine.States state = StateMachine.States.RESTING;
        arm.shoulder(Arm.Shoulder.FORWARDS);
        arm.extendo(Arm.Extendo.EXTENDED);
        arm.wrist(Arm.Wrist.FULL_DOWNWARDS);
        boolean snapshotPressed = false;
        boolean resetArm = false;
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            drive.updatePoseEstimate();
            if (gamepad1.left_bumper && snapshotPressed == false){
                snapshotPressed = true;
                webcam.updateCurrentDriveStage(Webcam.DRIVE_STAGE.MOVE_TO_TARGET);
                webcam.updateDriveStartPos(drive.pinpoint.getPositionRR());
                webcam.snapshot();
                arm.intake(Arm.Intake.INTAKE);
            }else if (!gamepad1.left_bumper){
                snapshotPressed = false;
            }
            if (gamepad1.right_bumper && resetArm == false){
                resetArm = true;
                webcam.updateCurrentDriveStage(Webcam.DRIVE_STAGE.DONE);
                arm.shoulder(Arm.Shoulder.FORWARDS);
                arm.extendo(Arm.Extendo.EXTENDED);
                arm.wrist(Arm.Wrist.FULL_DOWNWARDS);
                arm.clawRotate(Arm.ClawRotation.Horz1);
            }else if (!gamepad1.right_bumper){
                resetArm = false;
            }
            webcam.webcamDrive(drive,arm,teamColor,telemetry);
            if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE){
                driveTrain.run(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x,telemetry);
                if (arm.isLerpComplete()){
                    arm.intake(Arm.Intake.CLOSE);
                }
            }
            telemetry.addData("Current Drive pos",drive.pinpoint.getPositionRR());
            telemetry.addData("Set Drive pos",webcam.driveStartPos);
            arm.update(telemetry,teamColor);
            webcam.loop(telemetry);
            telemetry.update();
        }
    }
}
