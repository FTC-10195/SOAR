package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class VerticalSlides {
    //Set up your variables ex: DcMotor frontLeftMotor;
    DcMotor rightSlide;
    DcMotor leftSlide;
    int max = 900;
    int down = 30;
    int chamber = 350;
    public  void initiate(HardwareMap hardwareMap){
        rightSlide = hardwareMap.dcMotor.get("Right Slide");
        leftSlide = hardwareMap.dcMotor.get("Left Slide");
        rightSlide.setTargetPosition(0);
        leftSlide.setTargetPosition(0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void manual(double power, Telemetry telemetry){
       if (power == 0 && leftSlide.getCurrentPosition() > 80){
           power = .2;
       }
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        telemetry.addData("leftslidepos", leftSlide.getCurrentPosition());
        telemetry.addData("rightslidepos", rightSlide.getCurrentPosition());
    }
    public void reset(boolean options){
        if (options){
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
