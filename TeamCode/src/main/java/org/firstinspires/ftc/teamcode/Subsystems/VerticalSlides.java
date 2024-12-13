package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class VerticalSlides {
    public enum SlidePositions {
        DOWN,
        BUCKET,
        CHAMBER
    }
    SlidePositions slidePosition = SlidePositions.DOWN;
    DcMotor rightSlide;
    DcMotor leftSlide;
    int max = 720;
    int down = 0;
    int chamber = 350;
    double maxPower = 1;
    double lockPower = .1;
    boolean lock = false;
    int lockPosition;
    public  void initiate(HardwareMap hardwareMap){
        rightSlide = hardwareMap.dcMotor.get("Right Slide");
        leftSlide = hardwareMap.dcMotor.get("Left Slide");
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void update(){
        switch (slidePosition){
            case DOWN:
                leftSlide.setTargetPosition(down);
                rightSlide.setTargetPosition(-down);
                break;
            case BUCKET:
                leftSlide.setTargetPosition(max);
                rightSlide.setTargetPosition(-max);
                break;
            case CHAMBER:
                leftSlide.setTargetPosition(chamber);
                rightSlide.setTargetPosition(-chamber);
                break;
        }
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(maxPower);
        rightSlide.setPower(maxPower);
    }
    public void setSlidePosition(SlidePositions slidePosition){
        this.slidePosition = slidePosition;
    }
    public double error(){
        return Math.abs(leftSlide.getTargetPosition() - leftSlide.getCurrentPosition());
    }
    public void manual(double power,boolean overide, Telemetry telemetry){
       if (power > 0.1){
           leftSlide.setTargetPosition(max);
           rightSlide.setTargetPosition(-max);
           lock = false;
       } else if (power < -.1){
           leftSlide.setTargetPosition(0);
           rightSlide.setTargetPosition(0);
           lock = false;
       } else {
           if (!lock){
               lock = true;
               lockPosition = leftSlide.getCurrentPosition();
           }
           leftSlide.setTargetPosition(lockPosition);
           rightSlide.setTargetPosition(-lockPosition);
           if (!overide){
               power = lockPower;
           }
       }
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        telemetry.addData("leftslidepos", leftSlide.getCurrentPosition());
        telemetry.addData("rightslidepos", rightSlide.getCurrentPosition());
        telemetry.addData("targetPos", leftSlide.getTargetPosition());
        telemetry.addData("LockPos", lockPosition);
        telemetry.addData("Locked", lock);
        telemetry.addData("Power:", power);
    }
    public void reset(boolean options){
        if (options){
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return true;
            }
        };
    }
    public Action slideAction(SlidePositions state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setSlidePosition(state);
                return error() > 20;
            }
        };
    }
}
