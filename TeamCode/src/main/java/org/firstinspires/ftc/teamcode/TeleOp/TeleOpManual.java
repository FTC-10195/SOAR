package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;

@TeleOp
public class TeleOpManual extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;
        Arm arm = new Arm();
        DriveTrain driveTrain = new DriveTrain();
        VerticalSlides verticalSlides = new VerticalSlides();
        driveTrain.initiate(hardwareMap);
        verticalSlides.initiate(hardwareMap);
        arm.initiate(hardwareMap);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        boolean retracted = true;
        boolean resting = true;
        int intakePower = 0;
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            driveTrain.run(gamepad1.left_stick_x *1.1,-gamepad1.left_stick_y, -gamepad1.right_stick_x);
            verticalSlides.manual((gamepad1.right_trigger-gamepad1.left_trigger)/2,false, telemetry);
            verticalSlides.reset(gamepad1.options);
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                retracted = !retracted;
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                resting = !resting;
            }
            if (currentGamepad1.triangle && !previousGamepad1.triangle){
                if (intakePower == 0){
                    intakePower = 1;
                } else {
                    intakePower = 0;
                }
            }
            if (currentGamepad1.cross && !previousGamepad1.cross){
                if (intakePower == 0){
                    intakePower = -1;
                } else {
                    intakePower = 0;
                }
            }
            arm.extendo(retracted);
            //arm.shoulder(resting,telemetry);
            arm.intake(intakePower);
            telemetry.update();
            }
        }
    }
