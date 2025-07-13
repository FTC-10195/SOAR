package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.TeamColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

@TeleOp
public class VISIONPORTAL extends LinearOpMode {
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);
        Arm arm = new Arm();
        arm.initiate(hardwareMap);
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.initiate(hardwareMap);
        arm.shoulder(Arm.Shoulder.FORWARDS);
        arm.wrist(Arm.Wrist.DOWNWARDS);
        arm.extendo(Arm.Extendo.EXTENDED);
        arm.update(telemetry, TeamColor.Color.RED);
        waitForStart();
        while (opModeIsActive()) {
            arm.shoulder(Arm.Shoulder.FORWARDS);
            arm.wrist(Arm.Wrist.DOWNWARDS);
            arm.extendo(Arm.Extendo.EXTENDED);
            arm.update(telemetry, TeamColor.Color.RED);
            driveTrain.run(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
        }
    }
}
