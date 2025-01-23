package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@TeleOp
public class ResetSlides extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        while (opModeIsActive()) {
            verticalSlides.manual(0, true, telemetry);
            verticalSlides.reset(gamepad1.options);
            telemetry.update();
        }
    }
}
