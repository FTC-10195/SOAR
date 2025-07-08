package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamColor {
   public enum Color{
       RED,
       BLUE,
       YELLOW,
       BOTH,
       NONE
   }
   Color currentColor;
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
   public void status(Telemetry telemetry){
       telemetry.addData("TeamColor",currentColor);
   }
}
