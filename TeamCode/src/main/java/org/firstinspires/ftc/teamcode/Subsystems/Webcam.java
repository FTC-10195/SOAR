package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.opencv.core.Point;
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
    public static int CAMERA_WIDTH_PX = 320;
    public static int CAMERA_LENGTH_PX = 240;
    public static double LATERAL_OFFSET_PX = 0;
    public static double CAMERA_WIDTH_IN = 0;
    public static double CAMERA_LENGTH_IN = 0;
    public static double LATERAL_OFFSET_IN = 0;
    public static int MIN_SAMPLE_AREA_PX = 5000; //Filter out small blobs
    public static int MAX_SAMPLE_AREA_PX = 100000; //If it gets this close you're cooked

    public static double convertInchesToPx(double inches) {
        double px = inches;
        return px;
    }

    public static double convertPxToInches(double px) {
        double inches = px;
        return inches;
    }

    Point clawCenter = new Point(CAMERA_WIDTH_PX,CAMERA_LENGTH_PX + LATERAL_OFFSET_PX);

    double targetDistancePX = CAMERA_WIDTH_PX / 2; //Becomes the distance of the closest sample

    //Points for extremedies
    Point bottomLeft = new Point(Integer.MAX_VALUE, Integer.MIN_VALUE);
    Point bottomRight = new Point(Integer.MIN_VALUE, Integer.MIN_VALUE);
    Point topLeft = new Point(Integer.MAX_VALUE, Integer.MAX_VALUE);
    Point topRight = new Point(Integer.MIN_VALUE, Integer.MAX_VALUE);

    Point targetPos = clawCenter;
    public void initiate(HardwareMap hardwareMap, Telemetry telemetry) {
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(CAMERA_WIDTH_PX, CAMERA_LENGTH_PX))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    public void snapshot(Telemetry telemetry) {
        //Reset the variables everytime a snapshot is taken
        targetDistancePX = CAMERA_WIDTH_PX / 2;
        targetPos = clawCenter; //The vector 2 position on the camera where the sample is located
        bottomLeft = new Point(Integer.MAX_VALUE, Integer.MIN_VALUE);
        bottomRight = new Point(Integer.MIN_VALUE, Integer.MIN_VALUE);
        topLeft = new Point(Integer.MAX_VALUE, Integer.MAX_VALUE);
        topRight = new Point(Integer.MIN_VALUE, Integer.MAX_VALUE);

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_SAMPLE_AREA_PX, MAX_SAMPLE_AREA_PX, blobs);  // filter out very small blobs.
        telemetry.addLine(" Area Density Aspect  Center");
        // Display the size (area) and center location for each Blob.

        //Create variables for the TARGET SAMPLE
        double targetWidth = 0;
        double targetHeight = 0;
        Point[] myContourPoints;// A list of the many points within the blob
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            RotatedRect boxFit = blob.getBoxFit();
            //Variables FOR THIS BLOB
            double distanceXPx = ((CAMERA_WIDTH_PX / 2) - boxFit.center.x);
            double distanceYPx = ((CAMERA_LENGTH_PX / 2) - boxFit.center.y);
            double distancePx = Math.sqrt(Math.pow(distanceXPx, 2) + Math.pow(distanceYPx, 2));
            if (distancePx < targetDistancePX) {
                //Sets variables depending on what the target is
                targetDistancePX = distancePx;
                targetPos = boxFit.center;
                myContourPoints = blob.getContourPoints();
                int j = 0;
                for (Point thisContourPoint : myContourPoints) {
                    j += 1;
                    double x = thisContourPoint.x;
                    double y = thisContourPoint.y;

                    // Update bottom-left (smallest x, largest y)
                    if (x < bottomLeft.x || (x == bottomLeft.x && y > bottomLeft.y)) {
                        bottomLeft = thisContourPoint;
                    }

                    // Update bottom-right (largest x, largest y)
                    if (x > bottomRight.x || (x == bottomRight.x && y > bottomRight.y)) {
                        bottomRight = thisContourPoint;
                    }

                    // Update top-left (smallest x, smallest y)
                    if (x < topLeft.x || (x == topLeft.x && y < topLeft.y)) {
                        topLeft = thisContourPoint;
                    }

                    // Update top-right (largest x, smallest y)
                    if (x > topRight.x || (x == topRight.x && y < topRight.y)) {
                        topRight = thisContourPoint;
                    }
                }
            }
        }
    }
    public void loop(Telemetry telemetry){
        telemetry.addData("bottomLeft:",bottomLeft);
        telemetry.addData("bottomRight:",bottomRight);
        telemetry.addData("topLeft:",topLeft);
        telemetry.addData("topRight:",topRight);
        telemetry.addData("Distance from crosshair (px):",targetDistancePX);
        telemetry.addData("targetPos",targetPos);
        telemetry.addData("clawCenter",clawCenter);
    }
}
