package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class TeamColor {
   public enum Color{
       RED,
       BLUE,
       YELLOW,
       BOTH,
       NONE
   }
   Color currentColor;
   Servo rgbIndicator;
   Servo headlight;
   public static double headlightOn = .5;
   public static double headlightOff = 0;
   public static double redColor = 0.3;
    public static double blueColor = 0.65;
   public TeamColor(Color color){
       currentColor = color;
   }
   public void setColor(Color color){
       currentColor = color;
   }
   public Color getColor(){
       return currentColor;
   }
   public void flipColor(){
       switch (currentColor){
           case RED:
               setColor(Color.BLUE);
               break;
           case BLUE:
               setColor(Color.RED);
               break;
           case YELLOW:
               setColor(Color.YELLOW);
               break;
           case BOTH:
           case NONE:
               break;
       }
   }
   public void initiate(HardwareMap hardwareMap){
       headlight = hardwareMap.servo.get("headlight");
       rgbIndicator = hardwareMap.servo.get("rgb");
   }
   public void update(){
       headlight.setPosition(headlightOn);
       switch (currentColor){
           case RED:
               rgbIndicator.setPosition(redColor);
               break;
           case BLUE:
               rgbIndicator.setPosition(blueColor);
               break;
       }
   }
   public void status(Telemetry telemetry){
       telemetry.addData("TeamColor",currentColor);
       telemetry.addData("headlightPower", headlightOn);
   }
}
