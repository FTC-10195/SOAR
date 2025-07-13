package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.BarnacleCamera;
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
    BarnacleCamera barnacleCamera = new BarnacleCamera();
    Path noPath;
    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));  // Starting position
    private final Pose scorePose = new Pose(7.5, 122, Math.toRadians(315));
    private final Pose identifyPose = new Pose(14, 116, Math.toRadians(360));
    private final Pose rightGrab = new Pose(14, 109.5, Math.toRadians(360));
    private final Pose middleGrab = new Pose(14, 121, Math.toRadians(360));
    private final Pose leftGrab = new Pose(17, 119, Math.toRadians(380));
    private Path scorePreload, identify, park, rightStart, right, middleStart, middle, leftStart, left, score1, score2, score3, score4;
    private Point score3End;
    private Point sub1End;
    private Path sub1;
    //right1, middle2
    //right1, left2,
    //middle1, left3,
    long timeSnapshot = System.currentTimeMillis();


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        identify = new Path(new BezierLine(new Point(scorePose), new Point(identifyPose)));
        identify.setLinearHeadingInterpolation(scorePose.getHeading(), identifyPose.getHeading());

        rightStart = new Path(new BezierLine(new Point(identifyPose), new Point(rightGrab)));
        rightStart.setLinearHeadingInterpolation(identifyPose.getHeading(), rightGrab.getHeading());

        middleStart = new Path(new BezierLine(new Point(identifyPose), new Point(middleGrab)));
        middleStart.setLinearHeadingInterpolation(identifyPose.getHeading(), middleGrab.getHeading());

        leftStart = new Path(new BezierLine(new Point(identifyPose), new Point(leftGrab)));
        leftStart.setLinearHeadingInterpolation(identifyPose.getHeading(), leftGrab.getHeading());

        right = new Path(new BezierLine(new Point(scorePose), new Point(rightGrab)));
        right.setLinearHeadingInterpolation(scorePose.getHeading(), rightGrab.getHeading());


        middle = new Path(new BezierLine(new Point(scorePose), new Point(middleGrab)));
        middle.setLinearHeadingInterpolation(scorePose.getHeading(), middleGrab.getHeading());

        left = new Path(new BezierLine(new Point(scorePose), new Point(leftGrab)));
        left.setLinearHeadingInterpolation(scorePose.getHeading(), leftGrab.getHeading());

        sub1End = new Point(65.000, 102.000, Point.CARTESIAN);
        sub1 = new Path(new BezierCurve(
                new Point(12.5, 124, Point.CARTESIAN),
                new Point(62.13084112149532, 110.35514018691589, Point.CARTESIAN),
                sub1End
        )
        );
        sub1.setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()), Math.toRadians(275));


        score1 = new Path(new BezierLine(new Point(rightGrab), new Point(scorePose)));
        score1.setLinearHeadingInterpolation(rightGrab.getHeading(), scorePose.getHeading());

        score2 = new Path(new BezierLine(new Point(middleGrab), new Point(scorePose)));
        score2.setLinearHeadingInterpolation(middleGrab.getHeading(), scorePose.getHeading());

        score3 = new Path(new BezierLine(new Point(leftGrab), new Point(scorePose)));
        score3.setLinearHeadingInterpolation(leftGrab.getHeading(), scorePose.getHeading());

        score4 = new Path(
                new BezierCurve(
                        sub1End,
                        new Point(61.6822429906542, 109.23364485981308, Point.CARTESIAN),
                        new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
                )
        );
        score4.setLinearHeadingInterpolation(Math.toRadians(275), Math.toRadians(315));

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
                follower.followPath(identify);
                setPathState(pathState + 1);
                break;
            case 3:
                restSubsystems(1200, pathState);
                break;
            case 4:
                scoutSubsystems(500, pathState);
                barnacleCamera.identifyBarnacle();
                setPathState(pathState + 1);
                break;
            case 5:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                    case MIDDLE:
                        follower.followPath(rightStart);
                        break;
                    case RIGHT:
                        follower.followPath(middleStart);
                        break;
                }
                setPathState(pathState + 1);
                timeSnapshot = System.currentTimeMillis();
                break;
            case 6:
                scoutSubsystems(900, pathState);
                break;
            case 7:
                intakeSubsystems(900, pathState);
                break;
            case 8:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                    case MIDDLE:
                        follower.followPath(score1);
                        break;
                    case RIGHT:
                        follower.followPath(score2);
                        break;
                }
                setPathState(pathState + 1);

                break;
            case 9:
                scoreSubsystems(1850, pathState);
                break;
            case 10:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(middle);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        follower.followPath(left);
                        break;

                }
                setPathState(pathState + 1);
                break;
            case 11:
                restSubsystems(800, pathState);
                break;
            case 12:
                scoutSubsystems(1000, pathState);
                break;
            case 13:
                intakeSubsystems(1000, pathState);
                break;
            case 14:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(score2);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        follower.followPath(score3);
                        break;

                }
                setPathState(pathState + 1);
                break;
            case 15:
                scoreSubsystems(1800, pathState);
                break;
            case 16:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                    case MIDDLE:
                    case RIGHT:
                        follower.followPath(sub1);
                        break;

                }
                follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.1, 0, 0, 6, 0));
                barnacleCamera.clear();
                setPathState(pathState + 1);
                break;
            case 17:
                restSubsystems(1500, pathState);
                break;
            case 18:
                scoutSubsystems(1000000, pathState);
                double driveErrorX = Math.abs(sub1End.getX() - follower.getPose().getX());
                double driveErrorY = Math.abs(sub1End.getY() - follower.getPose().getY());
                if (driveErrorY < 2 && driveErrorX < 2) {
                    timeSnapshot = System.currentTimeMillis();
                    follower.breakFollowing();
                    setPathState(pathState + 1);
                }
                break;
            case 19:
                if (System.currentTimeMillis() - timeSnapshot > 800) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 20:
                webcam.setDriveStage(Webcam.DRIVE_STAGE.DRIVE);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 21:
                if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 22:
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    arm.intake(Arm.Intake.CLOSE);
                }
                if (System.currentTimeMillis() - timeSnapshot > 800){
                    follower.followPath(score4);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
            case 23:
                arm.shoulder(Arm.Shoulder.FORWARDS);
                if (System.currentTimeMillis() - timeSnapshot > 1300) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                } else {
                    arm.intake(Arm.Intake.CLOSE);
                }
                break;
            case 24:
                break;
        }


    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void scoreSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
        arm.shoulder(Arm.Shoulder.BUCKET);
        arm.wrist(Arm.Wrist.FORWARD);
        arm.clawRotate(Arm.ClawRotation.Horz1);
        arm.extendo(Arm.Extendo.RETRACTED);
        if (System.currentTimeMillis() - timeSnapshot > 400) {
            arm.extendo(Arm.Extendo.EXTENDED);
        }
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
        if (System.currentTimeMillis() - timeSnapshot > 500 && System.currentTimeMillis() - timeSnapshot < 800) {
            arm.intake(Arm.Intake.CLOSE);
        } else if (System.currentTimeMillis() - timeSnapshot > 800) {
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
            webcam.update(driveTrain, arm);
            autonomousPathUpdate();
            follower.update();
            if (webcam.currentDriveStage != Webcam.DRIVE_STAGE.DONE) {
                arm.intake(webcam.intakeState);
            }
            webcam.status(telemetry);

            telemetry.addData("Heading Error", follower.headingError);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
