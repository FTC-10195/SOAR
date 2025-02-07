package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.List;
@Config
@Disabled
public class Webcam {
    ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.RED)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .build();
    VisionPortal portal;
    int CAMERA_WIDTH = 320;
    int CAMERA_LENGTH = 240;
    double clawTarget = 0;
    boolean clawOveride = false;

  public void initiate(HardwareMap hardwareMap, Telemetry telemetry){
      portal = new VisionPortal.Builder()
              .addProcessor(colorLocator)
              .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_LENGTH))
              .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
              .build();
      telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates, Just use for debugging.
      telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
  }

  public void rotate(double servoAngle ,Telemetry telemetry){
      telemetry.addData("preview on/off", "... Camera Stream\n");

      // Read the current list
      List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
      ColorBlobLocatorProcessor.Util.filterByArea(3000, 500000, blobs);  // filter out very small blobs.
      telemetry.addLine(" Area Density Aspect  Center");
      // Display the size (area) and center location for each Blob.
      double xPos = 0;
      double yPos = 0;
      double selectedWidth = 0;
      double selectedHeight = 0;
      double closestDistance = 100;
      double targetAngle = 0;
      for(ColorBlobLocatorProcessor.Blob blob : blobs)
      {
//Lets try using blob.getContourPoints() to get the poisitiosn of the edges?
          RotatedRect boxFit = blob.getBoxFit();
          double width = boxFit.size.width;
          double height = boxFit.size.height;
          double distanceX =  ((CAMERA_WIDTH/2) - boxFit.center.x);
          double distanceY =  ((CAMERA_LENGTH/2) - boxFit.center.y);
          double distance = Math.sqrt(Math.pow(distanceX,2)+Math.pow(distanceY,2));
          if (distance < closestDistance){
              closestDistance = distance;
              xPos = boxFit.center.x;
              yPos = boxFit.center.y;
              selectedWidth = width;
              selectedHeight = height;
          }
          double angle = boxFit.angle;
          telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d) %5.2f %5.2f %5.2f %5.2f",
                  blob.getContourArea(), blob.getContourPoints(), blob.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y, distanceX,distanceY,distance,angle));
      }
      if (closestDistance < 100) {
          clawTarget = targetAngle;
          clawOveride = true;
      }else{
          clawOveride = false;
          clawTarget = 0.6;
      }
      telemetry.addData("targetAngle", targetAngle);
      telemetry.addData("clawTarget", clawTarget);
      telemetry.addData("targetOveride", clawOveride);
  }
  public double getClawRot(){
      return clawTarget;
  }
    public boolean getOveride(){
        return clawOveride;
    }
}
