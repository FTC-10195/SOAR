package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@Autonomous
public class AutoBasicSample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm();
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.initiate(hardwareMap);
        VerticalSlides.SlidePositions currentSlidePos = VerticalSlides.SlidePositions.DOWN;
        Arm.Intake intakePos = Arm.Intake.STOPPED;
        waitForStart();
        double startTime = 0;
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (startTime == 0){
                startTime = System.currentTimeMillis();
                arm.initiate(hardwareMap);
            }
            currentSlidePos = VerticalSlides.SlidePositions.BUCKET;
            if (Math.abs(startTime - System.currentTimeMillis()) < 3000) {
                driveTrain.run(0, .3,-0, telemetry);
            } else if (Math.abs(startTime - System.currentTimeMillis()) < 5000){
                intakePos = Arm.Intake.OUTTAKING;
            }else if (Math.abs(startTime - System.currentTimeMillis()) < 6000){
                driveTrain.run(0, -.3,-0, telemetry);
            } else if (Math.abs(startTime - System.currentTimeMillis()) > 7000) {
                driveTrain.run(0, 0,-0, telemetry);
                currentSlidePos = VerticalSlides.SlidePositions.DOWN;
                intakePos = Arm.Intake.STOPPED;
            }
            verticalSlides.setSlidePosition(currentSlidePos);
            arm.intake(intakePos);
            arm.update(telemetry,"");
            verticalSlides.update();
        }
    }
}
