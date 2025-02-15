package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Ascent;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
//import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@TeleOp
public class TeleOpWebcamTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
      //  Webcam webcam = new Webcam();
      //  webcam.initiate(hardwareMap,telemetry);
        waitForStart();
        if (isStopRequested()) return;


        DriveTrain driveTrain = new DriveTrain();
        driveTrain.initiate(hardwareMap);
        Webcam webcam = new Webcam();
        webcam.initiate(hardwareMap, Arm.TeamColor.RED,telemetry);
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
        arm.extendo(Arm.Extendo.EXTENDED);
        boolean snapshotPressed = false;
        boolean resetClaw = false;
        boolean rotationDown = false;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            if (gamepad1.left_bumper && snapshotPressed == false){
                snapshotPressed = true;
                webcam.snapshot();
            }else if (!gamepad1.left_bumper){
                snapshotPressed = false;
            }
            if (gamepad1.right_bumper && resetClaw == false){
                resetClaw = true;
                webcam.setClawRotation(Arm.ClawRotation.Horz1);
            }else if (!gamepad1.right_bumper){
                resetClaw = false;
            }
            if (gamepad1.left_trigger > 0.1 && rotationDown == false){
                rotationDown = true;
                if (arm.shoulderState == Arm.Shoulder.FORWARDS){
                    arm.shoulder(Arm.Shoulder.DOWNWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                }else {
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.FULL_DOWNWARDS);
                }
            }else if (gamepad1.left_trigger <= 0.1){
                rotationDown = false;
            }
            arm.clawRotate(webcam.sampleRotation);
            arm.update(telemetry,teamColor);
            webcam.loop(telemetry);
            telemetry.update();
        }
    }
}
