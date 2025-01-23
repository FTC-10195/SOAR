package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@Autonomous
public class AutoExtremelyBasicClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm();
        VerticalSlides verticalSlides = new VerticalSlides();
        verticalSlides.initiate(hardwareMap);
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.initiate(hardwareMap);
        waitForStart();
        double startTime = 0;
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (startTime == 0){
                startTime = System.currentTimeMillis();
            }
            if (Math.abs(startTime - System.currentTimeMillis()) < 300) {
                driveTrain.run(0, .3,-0, telemetry);
            }else if (Math.abs(startTime - System.currentTimeMillis()) < 2000) {
                driveTrain.run(.3, 0,-0, telemetry);
            } else if (Math.abs(startTime - System.currentTimeMillis()) < 3000) {
                driveTrain.run(0, -.3,-0, telemetry);
            }
        }
    }
}
