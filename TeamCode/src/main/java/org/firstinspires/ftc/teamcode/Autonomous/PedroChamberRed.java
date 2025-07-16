package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.BarnacleCamera;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class PedroChamberRed extends LinearOpMode {
    int pathState = 0;
    Timer pathTimer;
    Constants constants;
    Follower follower;
    TeamColor teamColor = new TeamColor(TeamColor.Color.RED);
    VerticalSlides verticalSlides = new VerticalSlides();
    Arm arm = new Arm();
    DriveTrain driveTrain = new DriveTrain();
    Webcam webcam = new Webcam();
    BarnacleCamera barnacleCamera = new BarnacleCamera();
    Path noPath;
    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));  // Starting position
    private final Pose scorePose = new Pose(7, 112, Math.toRadians(270));  // Starting position
    private Path scorePreload;
    //sub3 and sub4 are for when the barnacle is on the right
    //scoreRight 1 and 2 are for when barnacle is on the right
    //right1, middle2
    //right1, left2,
    //middle1, left3,
    long timeSnapshot = System.currentTimeMillis();


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    }

    public void autonomousPathUpdate() {
        double driveErrorX;
        double driveErrorY;
        switch (pathState) {
            case 0: // Move from start to scoring position
                //   scoreSubsystems(1000);
                follower.followPath(scorePreload);
                setPathState(1);
                timeSnapshot = System.currentTimeMillis();
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void scoreSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
        arm.shoulder(Arm.Shoulder.CHAMBER_SCORE);
        arm.wrist(Arm.Wrist.DOWNWARDS);
        arm.extendo(Arm.Extendo.RETRACTED);
        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis){
            arm.intake(Arm.Intake.DEPOSIT);
            setPathState(pathState+1);
            timeSnapshot = System.currentTimeMillis();
        }else{
            arm.intake(Arm.Intake.CLOSE);
        }
    }

    public void restSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
        arm.shoulder(Arm.Shoulder.UPWARDS);
        arm.extendo(Arm.Extendo.RETRACTED);
        arm.wrist(Arm.Wrist.FORWARD);
        arm.intake(Arm.Intake.CLOSE);
        arm.clawRotate(Arm.ClawRotation.Horz1);

        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis) {
            setPathState(pathState + 1);
            timeSnapshot = System.currentTimeMillis();
        }
    }

    public void scoutSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
        arm.shoulder(Arm.Shoulder.FORWARDS);

        arm.wrist(Arm.Wrist.DOWNWARDS);
        arm.intake(Arm.Intake.INTAKE);
        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis) {
            setPathState(pathState + 1);
            timeSnapshot = System.currentTimeMillis();
            arm.extendo(Arm.Extendo.EXTENDED);
            arm.shoulderLerpStartTime = System.currentTimeMillis();
        } else {
            arm.extendo(Arm.Extendo.EXTENDED);
        }
    }

    public void intakeSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
        arm.shoulder(Arm.Shoulder.DOWNWARDS);
        arm.wrist(Arm.Wrist.DOWNWARDS);
        arm.extendo(Arm.Extendo.EXTENDED);
        if (System.currentTimeMillis() - timeSnapshot > 500 && System.currentTimeMillis() - timeSnapshot < 700) {
            arm.intake(Arm.Intake.CLOSE);
        } else if (System.currentTimeMillis() - timeSnapshot > 700) {
            arm.intake(Arm.Intake.CLOSE);
            timeSnapshot = System.currentTimeMillis();
            setPathState(pathState + 1);
        } else {
            arm.intake(Arm.Intake.INTAKE);
        }
    }

    public void barnacleSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BARNACLE);
        arm.shoulder(Arm.Shoulder.FORWARDS);
        arm.extendo(Arm.Extendo.EXTENDED);
        arm.wrist(Arm.Wrist.DOWNWARDS);
        arm.intake(Arm.Intake.CLOSE);

        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis) {
            setPathState(pathState + 1);
            timeSnapshot = System.currentTimeMillis();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        arm.initiate(hardwareMap);
        verticalSlides.initiate(hardwareMap);
        webcam.initiate(hardwareMap, teamColor.getColor(), StateMachine.Mode.BUCKET, telemetry);
        driveTrain.initiate(hardwareMap);
        barnacleCamera.initiate(hardwareMap);
        pathTimer = new Timer();
        constants = new Constants();
        constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            verticalSlides.update();
            arm.update(telemetry, teamColor.getColor());
            autonomousPathUpdate();
            follower.update();
            if (webcam.currentDriveStage != Webcam.DRIVE_STAGE.DONE) {
                TelemetryPacket packet = new TelemetryPacket();
                arm.intake(webcam.intakeState);
                webcam.update(driveTrain, arm, packet);
            }
            webcam.status(telemetry);
            barnacleCamera.status(telemetry);
            telemetry.addData("Heading Error", follower.headingError);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
