package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@TeleOp
public class TeleOpBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Extendo extendo = new Extendo();
        DriveTrain driveTrain = new DriveTrain();
        Intake intake = new Intake();
        VerticalSlides verticalSlides = new VerticalSlides();
        Wrist wrist = new Wrist();
        waitForStart();
        if (isStopRequested()) return;
        extendo.initiate(hardwareMap);
        driveTrain.initiate(hardwareMap);
        intake.initiate(hardwareMap);
        verticalSlides.initiate(hardwareMap);
        wrist.initiate(hardwareMap);
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            extendo.run();
            driveTrain.run();
            intake.run();
            verticalSlides.run();
            wrist.run();
            telemetry.update();
        }
    }
}
