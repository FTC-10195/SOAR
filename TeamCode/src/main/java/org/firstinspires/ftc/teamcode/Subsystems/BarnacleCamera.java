package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

@Config
public class BarnacleCamera {

    public enum BarnacleLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static int white = 190;
    BarnacleLocations barnacleLocation = BarnacleLocations.LEFT;
    ColorBlobLocatorProcessor colorLocatorWhite = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(ColorSpace.RGB, new Scalar(white, white, white), new Scalar(255, 255, 255)))         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(false)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .build();
    VisionPortal portal;
    public static int CAMERA_WIDTH_PX = 320;
    public static int CAMERA_LENGTH_PX = 240;
    public static int MIN_SAMPLE_AREA_PX = 500; //Filter out small blobs
    public static int MAX_SAMPLE_AREA_PX = 100000; //If it gets this close you're cooked
    public static double leftX = 107;
    public static double middleX = 214;
    public static double rightX = CAMERA_WIDTH_PX;
    public static double bottomY = 320;
    public static double topY = 0;

    //Points for extremedies
    Point bottomLeft = null;
    Point bottomRight = null;
    Point topLeft = null;
    Point topRight = null;


    public void setLiveView(boolean On) {
        if (On) {
            portal.resumeLiveView();
        } else {
            portal.stopLiveView();
        }
    }

    public void identifyBarnacle() {
        //Identify white blobs
        List<ColorBlobLocatorProcessor.Blob> whiteBlobs;
        whiteBlobs = colorLocatorWhite.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_SAMPLE_AREA_PX, MAX_SAMPLE_AREA_PX, whiteBlobs);
        double barnacleSize = 0;
        int leftSize = 0;
        int middleSize = 0;
        int rightSize = 0;
        for (ColorBlobLocatorProcessor.Blob blob : whiteBlobs) {
            RotatedRect boxFit = blob.getBoxFit();
            if (boxFit.size.area() > barnacleSize){
                barnacleSize = boxFit.size.area();
                Point[] myContourPoints;// A list of the many points within the blob
                myContourPoints = blob.getContourPoints();
                for (Point p : myContourPoints) {
                    if (p.y < bottomY && p.y > topY){
                        return;
                    }
                    if (p.x < leftX){
                        leftSize +=1;
                    }else if (p.x > leftX && p.x < middleX){
                        middleSize += 1;
                    }else{
                        rightSize += 1;
                    }
                }
            }
        }
        if (leftSize > rightSize && leftSize > middleSize){
            barnacleLocation = BarnacleLocations.LEFT;
        }else if (rightSize > leftSize && rightSize > middleSize){
            barnacleLocation = BarnacleLocations.RIGHT;
        }else if (middleSize > leftSize && middleSize > rightSize){
            barnacleLocation = BarnacleLocations.MIDDLE;
        }
    }

    public BarnacleLocations getBarnacleLocation() {
        return barnacleLocation;
    }


    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public void initiate(HardwareMap hardwareMap) {
        portal = new VisionPortal.Builder()
                .addProcessors(colorLocatorWhite)
                .setCameraResolution(new Size(CAMERA_WIDTH_PX, CAMERA_LENGTH_PX))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .build();
    }

    public void status(Telemetry telemetry) {
        telemetry.addData("BARNACLE LOCATION", barnacleLocation);
    }


}
