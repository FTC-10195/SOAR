package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
//import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@TeleOp
public class BarnacleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
      //  Webcam webcam = new Webcam();
      //  webcam.initiate(hardwareMap,telemetry);
        waitForStart();
        if (isStopRequested()) return;
        TeamColor teamColor = new TeamColor(TeamColor.Color.RED);
        Webcam webcam = new Webcam();
        webcam.initiate(hardwareMap, teamColor.getColor(), StateMachine.Mode.CHAMBER,telemetry);
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        StateMachine.Mode mode = StateMachine.Mode.CHAMBER;
        StateMachine.States state = StateMachine.States.RESTING;
        boolean changeMode = false;
        boolean resetArm = false;
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            if (gamepad1.left_bumper){
                webcam.identifyBarnacle();
            }
            boolean switchMode = gamepad1.circle && !previousGamepad1.circle;
            boolean switchColor = gamepad1.cross && !previousGamepad1.cross;
            if (switchColor){
                teamColor.flipColor();
            }
            teamColor.status(telemetry);
            webcam.status(telemetry);
            telemetry.update();
        }
    }
}
