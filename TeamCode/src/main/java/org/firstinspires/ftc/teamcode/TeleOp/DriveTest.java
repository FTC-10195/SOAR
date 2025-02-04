package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@TeleOp
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;
        DriveTrain driveTrain = new DriveTrain();
        FtcDashboard dashboard;
        dashboard = FtcDashboard.getInstance();
        driveTrain.initiate(hardwareMap);
        while (opModeIsActive()) {
            driveTrain.reset(gamepad1.options);
            TelemetryPacket packet = new TelemetryPacket();
            driveTrain.testDrive(gamepad1.right_trigger - gamepad1.left_trigger);
            packet.put("fLPos", driveTrain.fLMPos());
            packet.put("bLPos", driveTrain.bLMPos());
            packet.put("fRPos", driveTrain.fRMPos());
            packet.put("bRPos", driveTrain.bRMPos());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
