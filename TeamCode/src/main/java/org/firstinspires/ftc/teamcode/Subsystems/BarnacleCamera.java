package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
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

    public static int white = 195;
    public static int blurSize = 7;
    BarnacleLocations barnacleLocation = BarnacleLocations.LEFT;
    ColorBlobLocatorProcessor colorLocatorWhite = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(ColorSpace.RGB, new Scalar(white, white, white), new Scalar(255, 255, 255)))         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(false)                        // Show contours on the Stream Preview
            .setBlurSize(blurSize)                               // Smooth the transitions between different colors in image
            .build();
    VisionPortal portal;
    public static int CAMERA_WIDTH_PX = 320;
    public static int CAMERA_LENGTH_PX = 240;
    public static int MIN_SAMPLE_AREA_PX = 250; //Filter out small blobs
    public static int MAX_SAMPLE_AREA_PX = 100000; //If it gets this close you're cooked
    public static double dividerLineXBucket = CAMERA_WIDTH_PX/2;
    public static double dividerLineXChamber = 180;
    double barnacleSize = 0;







   /* public void setLiveView(boolean On) {
        if (On) {
            portal.resumeLiveView();
        } else {
            portal.stopLiveView();
        }
    }

    */

    public void identifyBarnacleBucket() {
        //Identify white blobs
        barnacleSize = 250;
        barnacleLocation = BarnacleLocations.LEFT;
        List<ColorBlobLocatorProcessor.Blob> whiteBlobs;
        whiteBlobs = colorLocatorWhite.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_SAMPLE_AREA_PX, MAX_SAMPLE_AREA_PX, whiteBlobs);
        for (ColorBlobLocatorProcessor.Blob blob : whiteBlobs) {
            RotatedRect boxFit = blob.getBoxFit();
            if (boxFit.size.width * 5 < boxFit.size.height || boxFit.size.height * 5 < boxFit.size.width) {
                //Proportions are super out of whack like wth is this
                return;
            }
            if (boxFit.size.area() > barnacleSize) {
                barnacleSize = boxFit.size.area();
                if (boxFit.center.x > dividerLineXBucket) {
                    barnacleLocation = BarnacleLocations.RIGHT;
                } else {
                    barnacleLocation = BarnacleLocations.MIDDLE;
                }
            }

        }
    }
    public void identifyBarnacleChamber() {
        //Identify white blobs
        barnacleSize = 250;
        barnacleLocation = BarnacleLocations.RIGHT;
        List<ColorBlobLocatorProcessor.Blob> whiteBlobs;
        whiteBlobs = colorLocatorWhite.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_SAMPLE_AREA_PX, MAX_SAMPLE_AREA_PX, whiteBlobs);
        for (ColorBlobLocatorProcessor.Blob blob : whiteBlobs) {
            RotatedRect boxFit = blob.getBoxFit();
            if (boxFit.size.width * 5 < boxFit.size.height || boxFit.size.height * 5 < boxFit.size.width) {
                //Proportions are super out of whack like wth is this
                return;
            }
            if (boxFit.size.area() > barnacleSize) {
                barnacleSize = boxFit.size.area();
                if (boxFit.center.x > dividerLineXChamber) {
                    barnacleLocation = BarnacleLocations.MIDDLE;
                } else {
                    barnacleLocation = BarnacleLocations.LEFT;
                }
            }

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
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .build();
     //   FtcDashboard.getInstance().startCameraStream(portal, 0);
    }
    public void clear(){
        portal.close();
    }
    public void status(Telemetry telemetry) {
        telemetry.addData("BARNACLE LOCATION", barnacleLocation);
        telemetry.addData("BARNACLE SIZE", barnacleSize);
    }


}
