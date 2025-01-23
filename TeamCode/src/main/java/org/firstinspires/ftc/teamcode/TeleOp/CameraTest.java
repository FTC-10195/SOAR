package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@TeleOp
@Config
public class CameraTest extends LinearOpMode {
    public Point CameraCenter;
    public static Point Offset = new Point(0, 0);
    public int SCREEN_WIDTH;
    public int SCREEN_LENGTH;

    @Override
    public void runOpMode() {
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Define the ColorBlobLocatorProcessor
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        // Define the VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .addProcessor(new CustomPipeline())
                .setCameraResolution(new Size(320, 240))
                .setCamera(cameraName)
                .build();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeInInit()) {
            telemetry.addData("Status", "Initializing...");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(150, 20000, blobs);

            telemetry.addLine(" Area Density Aspect  Center");
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(),
                        (int) boxFit.center.x, (int) boxFit.center.y));
            }
            telemetry.update();
            sleep(50);
        }

        portal.close();
    }

    class CustomPipeline implements VisionProcessor {
        public Mat processFrame(Mat input) {
            SCREEN_WIDTH = input.cols();
            SCREEN_LENGTH = input.rows();
            CameraCenter = new Point(SCREEN_WIDTH / 2 + Offset.x, SCREEN_LENGTH / 2 + Offset.y);

            Imgproc.rectangle(input,
                    new Point(CameraCenter.x - 50, CameraCenter.y - 50),
                    new Point(CameraCenter.x + 50, CameraCenter.y + 50),
                    new Scalar(0, 255, 0),
                    2);

            return input;
        }

        @Override
        public void init(int i, int i1, CameraCalibration cameraCalibration) {

        }

        @Override
        public Object processFrame(Mat mat, long l) {
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

        }
    }
}
