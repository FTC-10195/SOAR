package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@Autonomous
public class AutoBucket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm();
        arm.initiate(hardwareMap);
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
                arm.extendo(Arm.Extendo.RETRACTED);
                arm.shoulder(Arm.Shoulder.UPWARDS);
                arm.clawRotate(Arm.ClawRotation.Horz1);
                arm.wrist(Arm.Wrist.FORWARD);
                arm.intake(Arm.Intake.CLOSE);
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
            }else if (System.currentTimeMillis() - startTime > 300 && System.currentTimeMillis() - startTime < 4000) {
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                arm.extendo(Arm.Extendo.CHAMBER);
                arm.shoulder(Arm.Shoulder.UPWARDS);
                arm.clawRotate(Arm.ClawRotation.Horz1);
                arm.wrist(Arm.Wrist.FORWARD);
                arm.intake(Arm.Intake.CLOSE);
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
            }else if (System.currentTimeMillis() - startTime > 4000 && System.currentTimeMillis() - startTime < 5000) {
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                arm.extendo(Arm.Extendo.CHAMBER);
                arm.shoulder(Arm.Shoulder.UPWARDS);
                arm.wrist(Arm.Wrist.FORWARD);
                arm.clawRotate(Arm.ClawRotation.Horz1);
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                arm.intake(Arm.Intake.DEPOSIT);
            }else if (System.currentTimeMillis() - startTime > 5000 && System.currentTimeMillis() - startTime < 10000) {
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                arm.extendo(Arm.Extendo.RETRACTED);
                arm.shoulder(Arm.Shoulder.UPWARDS);
                arm.wrist(Arm.Wrist.FORWARD);
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                arm.clawRotate(Arm.ClawRotation.Horz1);
                arm.intake(Arm.Intake.CLOSE);
            }else if (System.currentTimeMillis() - startTime > 10000 && System.currentTimeMillis() - startTime < 11000) {
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
                arm.extendo(Arm.Extendo.RETRACTED);
                arm.shoulder(Arm.Shoulder.UPWARDS);
                arm.clawRotate(Arm.ClawRotation.Horz1);
                arm.wrist(Arm.Wrist.FORWARD);
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                arm.intake(Arm.Intake.CLOSE);
            }
            verticalSlides.update();
            arm.update(telemetry, Arm.TeamColor.NONE);
        }
    }
}
