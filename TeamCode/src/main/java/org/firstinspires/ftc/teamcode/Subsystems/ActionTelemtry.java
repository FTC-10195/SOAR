package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionTelemtry {
    public Action telemetryAction(Telemetry telemetry) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.update();
                return true;
            }
        };
    }
    public Action telemetryAddAction(Telemetry telemetry,Arm.ClawRotation webcamClawRot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("Webcamclawrottrue",webcamClawRot);
                return true;
            }
        };
    }
}
