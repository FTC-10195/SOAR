package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;


import androidx.annotation.NonNull;

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
public class Webcam {
    public enum DRIVE_STAGE {
        DRIVE,
        DROP,
        DONE
    }


    public static int white = 190;
    public static int hMinYellow = 10;
    public static int sMinYellow = 100;
    public static int vMinYellow = 150;
    public static int hMaxYellow = 80;
    public static int sMaxYellow = 255;
    public static int vMaxYellow = 255;
    public static int blurSizeYellow = 3;
    public static int lowYRed = 0;
    public static int highYRed = 255;
    public static int lowCrRed = 140;
    public static int highCrRed = 255;
    public static int lowCbRed = 0;
    public static int highCbRed = 160;

    ColorBlobLocatorProcessor colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(
                    ColorSpace.YCrCb,
                    new Scalar(lowYRed, lowCrRed,  lowCbRed),
                    new Scalar(highYRed, highCrRed, highCbRed)
            ))
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .build();
    ColorBlobLocatorProcessor colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .build();
    ColorBlobLocatorProcessor colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(hMinYellow, sMinYellow, vMinYellow), new Scalar(hMaxYellow, sMaxYellow, vMaxYellow)))
            //new ColorRange(ColorSpace.HSV, new Scalar(hMinYellow, sMinYellow, vMinYellow), new Scalar(hMaxYellow, sMaxYellow, vMaxYellow)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(blurSizeYellow)                               // Smooth the transitions between different colors in image
            .build();
    ColorBlobLocatorProcessor colorLocatorWhite = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(ColorSpace.RGB, new Scalar(white, white, white), new Scalar(255, 255, 255)))         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
            .setDrawContours(false)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .build();
    ColorBlobLocatorProcessor colorLocatorTeam;
    VisionPortal portal;
    public DRIVE_STAGE currentDriveStage = DRIVE_STAGE.DONE;
    public Arm.Intake intakeState = Arm.Intake.INTAKE;
    public static long driveTargetTimeInMilis = 1000;
    public static int tolerancePID = 9;
    public static double kPX = 0.004525;
    public static double kIX = 0;
    public static double kDX = 0;
    public static double kFX = 0;
    public static double kPY = -0.0042;
    public static double kIY = 0;
    public static double kDY = 0;
    public static double kFY = 0;
    public static int CAMERA_WIDTH_PX = 320;
    public static int CAMERA_LENGTH_PX = 240;
    public static double LATERAL_OFFSET_PX = -40;
    public static double CAMERA_WIDTH_IN = 8.5;
    public static double CAMERA_LENGTH_IN = 6.5;
    public static double LATERAL_OFFSET_IN = -1.25; // Claw grabs about 1.25 inches on the y-axis below the center of camera
    public static int MIN_SAMPLE_AREA_PX = 500; //Filter out small blobs
    public static int MAX_SAMPLE_AREA_PX = 100000; //If it gets this close you're cooked
    public static double HORIZONTAL_OFFSET_PIX = 0;
    public static double MAX_SPEED = 0.2;

    PIDFController xPID = new PIDFController(kPX, kIX, kDX, kFX);
    PIDFController yPID = new PIDFController(kPY, kIY, kDY, kFY);

    public static double convertHorizontalPxToInches(double px) {
        double inches = px * (CAMERA_WIDTH_IN / CAMERA_WIDTH_PX);
        return inches;
    }

    public static double convertVerticalPxToInches(double px) {
        double inches = px * (CAMERA_LENGTH_IN / CAMERA_LENGTH_PX);
        return inches;
    }

    Point clawCenter = new Point(CAMERA_WIDTH_PX / 2 + HORIZONTAL_OFFSET_PIX, (CAMERA_LENGTH_PX / 2) + LATERAL_OFFSET_PX);

    double targetDistancePX = CAMERA_WIDTH_PX / 2; //Becomes the distance of the closest sample
    double targetDistanceX = CAMERA_WIDTH_PX / 2;
    double targetDistanceY = CAMERA_LENGTH_PX / 2;
    Vector2d targetVectorPx = new Vector2d(0, 0); //Becomes the x and y distances of the closest sample in PIXELS
    public static Vector2d targetVectorInches = new Vector2d(0, 0); //Becomes the x and y distances of the closest sample in INCHES
    public double angle = 0; //Angle of the target sample

    //Points for extremedies
    Point bottomLeft = null;
    Point bottomRight = null;
    Point topLeft = null;
    Point topRight = null;
    public Arm.ClawRotation sampleRotation = Arm.ClawRotation.Horz1;
    public Pose2d driveStartPos = new Pose2d(0, 0, 0);
    public long snapshotTime = System.currentTimeMillis();

    public void setDriveStage(DRIVE_STAGE currentDriveStage) {
        if (currentDriveStage == DRIVE_STAGE.DRIVE) {
            snapshotTime = System.currentTimeMillis();
        }
        this.currentDriveStage = currentDriveStage;
    }

    public void setLiveView(boolean On) {
        if (On) {
            portal.resumeLiveView();
        } else {
            portal.stopLiveView();
        }
    }

    public void updateDriveStartPos(Pose2d pos) {
        driveStartPos = pos;
    }


    public void update(DriveTrain driveTrain, Arm arm, TelemetryPacket packet) {
        xPID.setP(kPX);
        xPID.setI(kIX);
        xPID.setD(kDX);
        yPID.setP(kPY);
        yPID.setI(kIY);
        yPID.setD(kDY);
        switch (currentDriveStage) {
            case DRIVE:
                snapshot();
                intakeState = Arm.Intake.INTAKE;
                double xVec = xPID.calculate(clawCenter.x, targetPos.x);
                double yVec = yPID.calculate(clawCenter.y, targetPos.y);
                double speed = Math.sqrt(Math.pow(xVec, 2) + Math.pow(yVec, 2));
                packet.put("Old xVec", xVec);
                packet.put("Old yVec", yVec);
                packet.put("speed", speed);
                if (speed > MAX_SPEED) {
                    double greaterSpeed = Math.max(Math.abs(xVec), Math.abs(yVec));
                    double scaler = Math.abs(greaterSpeed / MAX_SPEED);
                    xVec /= scaler;
                    yVec /= scaler;
                }
                packet.put("New xVec", xVec);
                packet.put("New yVec", yVec);
                driveTrain.run(xVec, yVec, 0);

                if (
                        System.currentTimeMillis() - snapshotTime > driveTargetTimeInMilis ||
                                targetDistancePX < tolerancePID ||
                                (targetPos.x == clawCenter.x && targetPos.y == clawCenter.y)) {
                    snapshotTime = System.currentTimeMillis();
                    arm.shoulderLerpStartTime = snapshotTime;
                    arm.shoulder(Arm.Shoulder.DOWNWARDS);
                    setDriveStage(DRIVE_STAGE.DROP);
                }
                break;
            case DROP:
                driveTrain.run(0, 0, 0);
                arm.clawRotate(sampleRotation);
                if (System.currentTimeMillis() - snapshotTime > 400 && System.currentTimeMillis() - snapshotTime < 700) {
                    intakeState = Arm.Intake.CLOSE;
                } else if (System.currentTimeMillis() - snapshotTime > 700) {
                    setDriveStage(DRIVE_STAGE.DONE);
                } else {
                    intakeState = Arm.Intake.INTAKE;
                }

                break;
        }


    }

    Vector2d targetVec = new Vector2d(0, 0);

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

    public void setColorLocatorTeam(TeamColor.Color teamColor, boolean override) {
        if (teamColor == TeamColor.Color.RED) {
            colorLocatorTeam = colorLocatorRed;
        } else if (teamColor == TeamColor.Color.BLUE) {
            colorLocatorTeam = colorLocatorBlue;
        } else {
            colorLocatorTeam = colorLocatorYellow;
        }

    }

    boolean canSeeYellow = false;

    public void setColorLocatorMode(StateMachine.Mode mode, boolean override) {
        if (mode == StateMachine.Mode.BUCKET) {
            canSeeYellow = true;
        } else {
            canSeeYellow = false;
        }
    }

    public void initiate(HardwareMap hardwareMap, TeamColor.Color teamColor, StateMachine.Mode mode, Telemetry telemetry) {
        portal = new VisionPortal.Builder()
                .addProcessors(colorLocatorRed, colorLocatorBlue, colorLocatorYellow)
                .setCameraResolution(new Size(CAMERA_WIDTH_PX, CAMERA_LENGTH_PX))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(false)
                .build();
        FtcDashboard.getInstance().startCameraStream(portal, 0);
        setColorLocatorTeam(teamColor, true);
        setColorLocatorMode(mode, true);
        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    public void findTargetBlob(List<ColorBlobLocatorProcessor.Blob> blobs) {
        Point[] myContourPoints;// A list of the many points within the blob
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            //Variables FOR THIS BLOB
            double distanceXPx;
            double distanceYPx;
            double distancePx;
            double sumX = 0, sumY = 0;
            myContourPoints = blob.getContourPoints();
            for (Point p : myContourPoints) {
                sumX += p.x;
                sumY += p.y;
            }
            double count = myContourPoints.length;
            Point centroid = new Point((double) (sumX / count), (double) (sumY / count));

            distanceYPx = (clawCenter.y - centroid.y);
            distanceXPx = (clawCenter.x - centroid.x);
            distancePx = Math.sqrt(Math.pow(distanceXPx, 2) + Math.pow(distanceYPx, 2));
            if (distancePx < targetDistancePX && distanceXPx != 10000 && distanceYPx != 1000) {
                //Sets variables depending on what the target is
                targetDistancePX = distancePx;
                targetVectorPx = new Vector2d(distanceXPx, distanceYPx);
                targetDistanceX = distanceXPx;
                targetDistanceY = distanceYPx;
                targetVectorInches = new Vector2d(-convertHorizontalPxToInches(distanceXPx), convertVerticalPxToInches(distanceYPx));
                targetPos = centroid;


                for (Point p : myContourPoints) {

                    double x = p.x, y = p.y;

                    // Compute helper values relative to the centroid
                    if (x <= centroid.x && y <= centroid.y) {  // Top-left quadrant
                        if (topLeft == null || (x + y < topLeft.x + topLeft.y)) {
                            topLeft = p;
                        }
                    } else if (x >= centroid.x && y <= centroid.y) {  // Top-right quadrant
                        if (topRight == null || (x - y > topRight.x - topRight.y)) {
                            topRight = p;
                        }
                    } else if (x <= centroid.x && y >= centroid.y) {  // Bottom-left quadrant
                        if (bottomLeft == null || (x - y < bottomLeft.x - bottomLeft.y)) {
                            bottomLeft = p;
                        }
                    } else if (x >= centroid.x && y >= centroid.y) {  // Bottom-right quadrant
                        if (bottomRight == null || (x + y > bottomRight.x + bottomRight.y)) {
                            bottomRight = p;
                        }
                    }
                }
                //This shouldn't happen but if it does, just have the claw rotate horizontally
                if (bottomRight == null || bottomLeft == null || topRight == null || topLeft == null) {
                    sampleRotation = Arm.ClawRotation.Horz1;
                    return;
                }


                // Find the angle of the longest side
                angle = getRectangleAngle(topLeft.x, topLeft.y, topRight.x, topRight.y, bottomRight.x, bottomRight.y, bottomLeft.x, bottomLeft.y);
                //Translate to sample rotation
                double error = 22.5;
                if ((angle < (0 + (error)) && angle > (0 - (error))) || (angle < (180 + (error)) && angle > (180 - error)) || (angle < (-180 + (error)) && angle > (-180 - (error)))) {
                    //Piece is horizontal, claw should be vertical
                    sampleRotation = Arm.ClawRotation.Vert;
                } else if ((angle < (90 + (error)) && angle > (90 - (error))) || (angle < (270 + (error)) && angle > (270 - (error))) || (angle < (-90 + (error)) && angle > (-90 - (error))) || (angle < (-270 + (error)) && angle > (-270 - (error)))) {
                    sampleRotation = Arm.ClawRotation.Horz1;
                } else if ((angle > (45 - (error)) && angle < (45 + (error))) || (angle < (-315 + (error)) && angle > (-315 - (error))) || (angle < (225 + (error)) && angle > (225 - (error))) || (angle < (-135 + (error)) && angle > (-135 - (error)))) {
                    sampleRotation = Arm.ClawRotation.Diag1;
                } else if ((angle > (135 - (error)) && angle < (135 + (error))) || (angle < (-225 + (error)) && angle > (-225 - (error))) || (angle < (315 + (error)) && angle > (315 - (error))) || (angle < (-45 + (error)) && angle > (-45 - (error)))) {
                    sampleRotation = Arm.ClawRotation.Diag2;
                }
            }
        }
    }


    public void snapshot() {
        //Reset the variables everytime a snapshot is taken
        targetDistancePX = 10000;
        targetDistancePX = 10000;
        targetDistanceY = 10000;
        targetVectorInches = new Vector2d(0, 0);
        targetPos = clawCenter; //The vector 2 position on the camera where the sample is located
        bottomLeft = null;
        bottomRight = null;
        topLeft = null;
        topRight = null;

        // Read the current list, look for yellow ones first
        if (canSeeYellow) {
            List<ColorBlobLocatorProcessor.Blob> blobsYellow = colorLocatorYellow.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(MIN_SAMPLE_AREA_PX, MAX_SAMPLE_AREA_PX, blobsYellow);  // filter out very small blobs.
            findTargetBlob(blobsYellow);
        }
        List<ColorBlobLocatorProcessor.Blob> blobsTeam = colorLocatorTeam.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_SAMPLE_AREA_PX, MAX_SAMPLE_AREA_PX, blobsTeam);  // filter out very small blobs.
        findTargetBlob(blobsTeam);

    }

    public void status(Telemetry telemetry) {
        telemetry.addData("DRIVE STAGE", currentDriveStage);
     /*   telemetry.addData("PX Distance Vec", targetVectorPx);
        telemetry.addData("Inches Distance Vec", targetVectorInches);
        telemetry.addData("Rotation",sampleRotation);
        telemetry.addData("angle",angle);
        telemetry.addData("bottomLeft:",bottomLeft);
        telemetry.addData("bottomRight:",bottomRight);
        telemetry.addData("topLeft:",topLeft);
        telemetry.addData("topRight:",topRight);
        telemetry.addData("Distance from crosshair (px):",targetDistancePX);
        telemetry.addData("targetPos",targetPos);
        telemetry.addData("clawCenter",clawCenter);
      */
        telemetry.addData("colorLocatorTeam", colorLocatorTeam);
    }
    public void statusFTCDashboard(TelemetryPacket packet){
        packet.put("Webcam error", targetDistancePX);
        packet.put("Webcam error X", targetDistanceX);
        packet.put("Webcam error Y", targetDistanceY);
    }

    public void setClawRotation(Arm.ClawRotation rot) {
        sampleRotation = rot;
    }

    public Action updateAction(Telemetry telemetry) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                status(telemetry);
                return true;
            }
        };
    }

    public Action setClawRotation(Arm arm, boolean rotate) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (rotate) {
                    arm.clawRotate(sampleRotation);
                }
                return false;
            }
        };
    }

    public Action setDrive(PinpointDrive drive, Pose2d pos) {
        targetVec = new Vector2d(pos.position.x + targetVectorInches.x, pos.position.y + targetVectorInches.y);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.actionBuilder(pos)
                        .strafeToConstantHeading(targetVec)
                        .build();
                return false;
            }
        };
    }

    public Action snapshotAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                snapshot();
                return false;
            }
        };
    }
}
