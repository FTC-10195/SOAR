package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class PedroSample extends LinearOpMode {
    int pathState = 0;
    Timer pathTimer;
    Constants constants;
    Follower follower;
    TeamColor teamColor = new TeamColor(TeamColor.Color.RED);
    VerticalSlides verticalSlides = new VerticalSlides();
    Arm arm = new Arm();
    DriveTrain driveTrain = new DriveTrain();
    Webcam webcam = new Webcam();
    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));  // Starting position
    private final Pose scorePose = new Pose(8, 124, Math.toRadians(315));
    private final Pose barnicleIdentificationPose = new Pose(12, 117.5, Math.toRadians(360));
    private final Pose rightGrab = new Pose(12, 115, Math.toRadians(360));
    private final Pose middleGrab = new Pose(12, 119, Math.toRadians(360));
    private final Pose leftGrab = new Pose(12, 119, Math.toRadians(405));
    private Path scorePreload, park, barnicleIdentification, right1, middle1, middle2, left2, left3;
    //right1, middle2
    //right1, left2,
    //middle1, left3,
    long timeSnapshot = System.currentTimeMillis();

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        barnicleIdentification = new Path(new BezierLine(new Point(scorePose), new Point(barnicleIdentificationPose)));
        barnicleIdentification.setLinearHeadingInterpolation(scorePose.getHeading(), barnicleIdentificationPose.getHeading());

        right1 = new Path(new BezierLine(new Point(barnicleIdentificationPose), new Point(rightGrab)));
        right1.setLinearHeadingInterpolation(barnicleIdentificationPose.getHeading(), rightGrab.getHeading());

        middle2 = new Path(new BezierLine(new Point(rightGrab), new Point(middleGrab)));
        middle2.setLinearHeadingInterpolation(rightGrab.getHeading(), middleGrab.getHeading());

        middle1 = new Path(new BezierLine(new Point(barnicleIdentificationPose), new Point(middleGrab)));
        middle1.setLinearHeadingInterpolation(barnicleIdentificationPose.getHeading(), middleGrab.getHeading());

        left2 = new Path(new BezierLine(new Point(rightGrab), new Point(leftGrab)));
        left2.setLinearHeadingInterpolation(rightGrab.getHeading(), leftGrab.getHeading());

        left2 = new Path(new BezierLine(new Point(middleGrab), new Point(leftGrab)));
        left2.setLinearHeadingInterpolation(middleGrab.getHeading(), leftGrab.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                //   scoreSubsystems(1000);
                follower.followPath(scorePreload);
                setPathState(1);
                timeSnapshot = System.currentTimeMillis();
                break;
            case 1: // Move from scoring to barnacle identification position
                scoreSubsystems(1350, pathState);
                break;
            case 2:
                follower.followPath(barnicleIdentification);
                restSubsystems(800,pathState);
                break;
            case 3:
                barnacleSubsystems(1200, pathState);
            case 4:
                webcam.identifyBarnacle();
                pathState += 1;
                break;
            case 5:
                switch (webcam.getBarnacleLocation()) {
                    case LEFT:
                        scoutSubsystems(400, pathState);
                        follower.followPath(right1);
                }
      /*
            case 2:
                switch (webcam.getBarnacleLocation()) {
                    case LEFT:
                        barnacleLeft();
                        break;
                    case MIDDLE:
                        barnacleMiddle();
                        break;
                    case RIGHT:
                        barnacleRight();
                        break;
                }
                setPathState(1);
                break;

       */
        }


    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void scoreSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
        arm.shoulder(Arm.Shoulder.BUCKET);
        arm.extendo(Arm.Extendo.EXTENDED);
        arm.wrist(Arm.Wrist.FORWARD);


        if (System.currentTimeMillis() - timeSnapshot > (timeToWaitMilis)) {
            arm.intake(Arm.Intake.DEPOSIT);
            setPathState(pathState + 1);
            timeSnapshot = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - timeSnapshot > (timeToWaitMilis - 300)) {
            arm.intake(Arm.Intake.DEPOSIT);
        } else {
            arm.intake(Arm.Intake.CLOSE);
        }
    }

    public void restSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
        arm.shoulder(Arm.Shoulder.UPWARDS);
        arm.extendo(Arm.Extendo.RETRACTED);
        arm.wrist(Arm.Wrist.FORWARD);
        arm.intake(Arm.Intake.CLOSE);

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
        if (System.currentTimeMillis() - timeSnapshot < 200) {
            arm.extendo(Arm.Extendo.RETRACTED);
        } else if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis) {
            setPathState(pathState + 1);
            timeSnapshot = System.currentTimeMillis();
            arm.extendo(Arm.Extendo.EXTENDED);
        } else {
            arm.extendo(Arm.Extendo.EXTENDED);
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
            webcam.update(driveTrain, arm);
            autonomousPathUpdate();
            follower.update();
            webcam.status(telemetry);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
