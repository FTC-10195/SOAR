package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

@Autonomous
public class AutonomousBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain();
        waitForStart();
        if (isStopRequested()) return;
        driveTrain.initiate(hardwareMap);
        while (opModeIsActive()){
            driveTrain.run(0,.2,0);
        }
    }
}
