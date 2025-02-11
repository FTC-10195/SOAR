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
    public static int MIN_SAMPLE_AREA_PX = 500; //Filter out small blobs
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
    public double angle = 0; //Angle of the target sample

    //Points for extremedies
    Point bottomLeft =null;
    Point bottomRight = null;
    Point topLeft = null;
    Point topRight = null;
    public Arm.ClawRotation sampleRotation = Arm.ClawRotation.Horz1;
    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    public static double getRectangleAngle(double x1, double y1, double x2, double y2,
                                           double x3, double y3, double x4, double y4) {
        // Store all edges with their distances
        double[][] edges = {
                {x1, y1, x2, y2, distance(x1, y1, x2, y2)},
                {x2, y2, x3, y3, distance(x2, y2, x3, y3)},
                {x3, y3, x4, y4, distance(x3, y3, x4, y4)},
                {x4, y4, x1, y1, distance(x4, y4, x1, y1)}
        };

        // Find the longest edge
        double[] longestEdge = edges[0];
        for (double[] edge : edges) {
            if (edge[4] > longestEdge[4]) {
                longestEdge = edge;
            }
        }
        double lx1 = longestEdge[0], ly1 = longestEdge[1];
        double lx2 = longestEdge[2], ly2 = longestEdge[3];

        // Compute angle using atan2
        return Math.toDegrees(Math.atan2(ly2 - ly1, lx2 - lx1));
    }

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
        bottomLeft = null;
        bottomRight = null;
        topLeft = null;
        topRight = null;

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_SAMPLE_AREA_PX, MAX_SAMPLE_AREA_PX, blobs);  // filter out very small blobs.

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

                double sumX = 0, sumY = 0;
                double count = myContourPoints.length;

                for (Point p : myContourPoints) {
                    sumX += p.x;
                    sumY += p.y;
                }

                Point centroid = new Point((double) (sumX / count), (double) (sumY / count));

                for (Point p : myContourPoints) {

                    double x = p.x, y = p.y;

                    // Compute helper values relative to the centroid
                    if (x <= centroid.x && y <= centroid.y) {  // Top-left quadrant
                        if (topLeft == null || (x + y < topLeft.x + topLeft.y)) {
                            topLeft = p;
                        }
                    }
                    else if (x >= centroid.x && y <= centroid.y) {  // Top-right quadrant
                        if (topRight == null || (x - y > topRight.x - topRight.y)) {
                            topRight = p;
                        }
                    }
                    else if (x <= centroid.x && y >= centroid.y) {  // Bottom-left quadrant
                        if (bottomLeft == null || (x - y < bottomLeft.x - bottomLeft.y)) {
                            bottomLeft = p;
                        }
                    }
                    else if (x >= centroid.x && y >= centroid.y) {  // Bottom-right quadrant
                        if (bottomRight == null || (x + y > bottomRight.x + bottomRight.y)) {
                            bottomRight = p;
                        }
                    }
                }

                // Find the angle of the longest side
                angle = getRectangleAngle(topLeft.x, topLeft.y, topRight.x, topRight.y, bottomRight.x, bottomRight.y, bottomLeft.x, bottomLeft.y);
                //Translate to sample rotation
                double error = 30;
                if ((angle < (0 + (error/2)) && angle > (0 - (error/2))) || (angle < (180 + (error/2)) && angle > (180-error/2)) || (angle < (-180 + (error/2)) && angle > (-180 - (error/2)))){
                    //Piece is horizontal, claw should be vertical
                    sampleRotation = Arm.ClawRotation.Vert;
                }else if ((angle < (90 + (error/2)) && angle > (90 - (error/2))) || (angle < (270 + (error/2)) && angle > (270 - (error/2))) || (angle < (-90 + (error/2)) && angle > (-90 - (error/2))) || (angle < (-270 + (error/2)) && error > (-270 - (error/2)))){
                    sampleRotation = Arm.ClawRotation.Horz1;
                }else if ((angle < (45 - (error/2)) && angle > (45 + (error))) || (angle < (-315 + (error)) && angle > (-315 - (error))) || (angle < (225 + (error)) && angle > (225 - (error))) || (angle < (-135 + (error)) && angle > (-135 - (error)))){
                    sampleRotation = Arm.ClawRotation.Diag1;
                }else if ((angle < (135 - (error/2)) && angle > (135 + (error))) || (angle < (-225 + (error)) && angle > (-225 - (error))) || (angle < (315 + (error)) && angle > (315 - (error))) || (angle < (-45 + (error)) && angle > (-45 - (error)))){
                    sampleRotation = Arm.ClawRotation.Diag2;
                }
            }
        }
    }
    public void loop(Telemetry telemetry){
        telemetry.addData("Rotation",sampleRotation);
        telemetry.addData("angle",angle);
        telemetry.addData("bottomLeft:",bottomLeft);
        telemetry.addData("bottomRight:",bottomRight);
        telemetry.addData("topLeft:",topLeft);
        telemetry.addData("topRight:",topRight);
        telemetry.addData("Distance from crosshair (px):",targetDistancePX);
        telemetry.addData("targetPos",targetPos);
        telemetry.addData("clawCenter",clawCenter);
    }
}
