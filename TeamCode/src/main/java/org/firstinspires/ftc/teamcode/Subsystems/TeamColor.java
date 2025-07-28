package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class TeamColor {
   public enum Color{
       RED,
       BLUE,
       YELLOW,
       WHITE,
       BOTH,
       NONE
   }
   Color currentColor;
   Servo rgbIndicator;
   Servo headlight;
   public static double headlightOn = .5;
   public static double headlightOff = 0;
   public static double redColor = 0.279;
    public static double blueColor = 0.611;
    public static double whiteColor = 1;
    public static double yellowColor = .34;
    public static double greenColor = .5;
    public boolean unclipMode = false;
    public boolean bucektMode = false;
    public boolean yellowUse = false;
    public static int flashTime = 600;
    long timeSnapshot = System.currentTimeMillis();
   public TeamColor(Color color){
       currentColor = color;
       timeSnapshot = System.currentTimeMillis();
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
               setColor(Color.WHITE);
               break;
           case WHITE:
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
       switch (currentColor){
           case RED:
               rgbIndicator.setPosition(redColor);
               break;
           case BLUE:
               rgbIndicator.setPosition(blueColor);
               break;
           case YELLOW:
               rgbIndicator.setPosition(yellowColor);
               break;
           case WHITE:
               rgbIndicator.setPosition(whiteColor);
               break;
           case NONE:
               rgbIndicator.setPosition(0);
       }
       if (unclipMode){
           rgbIndicator.setPosition(greenColor);
       }
       if (bucektMode && !currentColor.equals(Color.WHITE)){
           if (System.currentTimeMillis() - timeSnapshot > flashTime){
               timeSnapshot = System.currentTimeMillis();
               yellowUse = !yellowUse;
           }
       }else{
           yellowUse = false;
       }
       if (yellowUse){
           rgbIndicator.setPosition(yellowColor);
       }
   }
   public void runHeadlights(boolean run){
       if (run){
           headlight.setPosition(headlightOn);
       }else{
           headlight.setPosition(headlightOff);
       }
   }
   public void status(Telemetry telemetry){
       telemetry.addData("TeamColor",currentColor);
       telemetry.addData("headlightPower", headlightOn);
   }
}
