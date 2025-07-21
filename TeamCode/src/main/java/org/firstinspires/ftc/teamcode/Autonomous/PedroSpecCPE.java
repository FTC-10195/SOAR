package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
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
public class PedroSpecCPE extends LinearOpMode {
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
    private final Pose startPose = new Pose(7, 64, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(43, 64, Math.toRadians(0));  // Starting position
    private final Pose humanIntakePose = new Pose(11, 35, Math.toRadians(0));
    private final Pose identifyPose = new Pose(21.5, 43.2, Math.toRadians(-38));
    private final Pose middleGrabPose = new Pose(22, 32.5, Math.toRadians(-38));
    private final Pose rightGrabPose = new Pose(22, 23.2, Math.toRadians(-38));
    private final Pose depositLeftPose = new Pose(21.5, identifyPose.getY() + 3, Math.toRadians(-125));
    private final Pose depositMiddlePose = new Pose(humanIntakePose.getX() + 7, middleGrabPose.getY() - 3, Math.toRadians(0));
    private final Pose depositRightPose = new Pose(humanIntakePose.getX() + 7, rightGrabPose.getY() - 3, Math.toRadians(0));
    private final Pose humanIntakeMiddlePose = new Pose(humanIntakePose.getX(), depositMiddlePose.getY(), Math.toRadians(0));
    private final Pose humanIntakeRightPose = new Pose(humanIntakePose.getX(), depositRightPose.getY(), Math.toRadians(0));
    private Path scorePreload, grabLeft, grabMiddle, grabRight, depositLeft, depositMiddle, depositRight, humanIntakeFirst, scoreFirst, scoreSecond, scoreThird, humanIntakeSecond, humanIntakeThird;


    long timeSnapshot = System.currentTimeMillis();


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(3);

        grabLeft = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(10.409, 64.483, Point.CARTESIAN),
                new Point(identifyPose.getX(), identifyPose.getY(), Point.CARTESIAN)
        ));
        grabLeft.setLinearHeadingInterpolation(scorePose.getHeading(), identifyPose.getHeading());


        //BARNACLE LEFT;

        grabRight = new Path(new BezierCurve(new Point(middleGrabPose),
                new Point(identifyPose),
                new Point(rightGrabPose)
        )
        );
        grabRight.setLinearHeadingInterpolation(middleGrabPose.getHeading(), rightGrabPose.getHeading());

        depositRight = new Path(new BezierLine(new Point(rightGrabPose), new Point(depositRightPose)));
        depositRight.setLinearHeadingInterpolation(rightGrabPose.getHeading(), depositRightPose.getHeading());

        humanIntakeFirst = new Path(new BezierLine(new Point(depositRightPose), new Point(humanIntakeRightPose)));
        humanIntakeFirst.setLinearHeadingInterpolation(depositRightPose.getHeading(), humanIntakeRightPose.getHeading());

        depositLeft = new Path(new BezierLine(new Point(identifyPose), new Point(depositLeftPose)));
        depositLeft.setLinearHeadingInterpolation(identifyPose.getHeading(), depositLeftPose.getHeading());

        grabMiddle = new Path(new BezierLine(new Point(depositLeftPose), new Point(middleGrabPose.getX(), middleGrabPose.getY())));
        grabMiddle.setLinearHeadingInterpolation(depositLeftPose.getHeading(), middleGrabPose.getHeading());

        depositMiddle = new Path(new BezierLine(new Point(middleGrabPose), new Point(depositMiddlePose)));
        depositMiddle.setLinearHeadingInterpolation(middleGrabPose.getHeading(), depositMiddlePose.getHeading());

        scoreFirst = new Path(new BezierCurve(
                new Point(humanIntakeRightPose.getX(), humanIntakeRightPose.getY(), Point.CARTESIAN),
                new Point(30, 33, Point.CARTESIAN),
                new Point(17, 64, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN)
        ));
        scoreFirst.setLinearHeadingInterpolation(humanIntakeRightPose.getHeading(), scorePose.getHeading());
        scoreFirst.setZeroPowerAccelerationMultiplier(1);


        humanIntakeSecond = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN),
                new Point(26, 64, Point.CARTESIAN),
                new Point(40, 33, Point.CARTESIAN),
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN)
        ));
        humanIntakeSecond.setLinearHeadingInterpolation(scorePose.getHeading(), humanIntakePose.getHeading());

        humanIntakeThird = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY() + 4, Point.CARTESIAN),
                new Point(26, 64, Point.CARTESIAN),
                new Point(40, 33, Point.CARTESIAN),
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN)
        ));
        humanIntakeThird.setLinearHeadingInterpolation(scorePose.getHeading(), humanIntakePose.getHeading());

        scoreSecond = new Path(new BezierCurve(
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN),
                new Point(22, 62, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 4, Point.CARTESIAN)
        ));
        scoreSecond.setLinearHeadingInterpolation(humanIntakePose.getHeading(), scorePose.getHeading());

        scoreThird = new Path(new BezierCurve(
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN),
                new Point(22, 62, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 6, Point.CARTESIAN)
        ));
        scoreThird.setLinearHeadingInterpolation(humanIntakePose.getHeading(), scorePose.getHeading());

    }

    public void autonomousPathUpdate() {
        double driveErrorX;
        double driveErrorY;
        switch (pathState) {
            case 0: // Move from start to scoring position
                //   scoreSubsystems(1000);
                timeSnapshot = System.currentTimeMillis();
                scoreSubsystems(100000000, pathState);
                arm.intake(Arm.Intake.CLOSE);
                setPathState(1);
                break;
            case 1:
                if (System.currentTimeMillis() - timeSnapshot > 700) {

                    arm.intake(Arm.Intake.CLOSE);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 2:
                follower.followPath(scorePreload);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                arm.intake(Arm.Intake.CLOSE);
                //  restSubsystems(1000,pathState);
                break;
            case 3:
                scoreSubsystems(1400, pathState);
                break;
            case 4:
                follower.followPath(grabLeft);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 5:
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 6:
                restSubsystems(1000, pathState);
                break;
            case 7:
                scoutSubsystems(800, pathState);
                arm.clawRotate(Arm.ClawRotation.LEFTDIAG);
                break;
            case 8:
                intakeSubsystems(900, pathState);
                break;
            case 9:
                follower.followPath(depositLeft);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                arm.shoulder(Arm.Shoulder.FORWARDS);
            case 10:
                if (System.currentTimeMillis() - timeSnapshot > 1200) {
                    arm.intake(Arm.Intake.DEPOSIT);
                    follower.followPath(grabMiddle);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 11:
                if (System.currentTimeMillis() - timeSnapshot > 1300) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 12:
                intakeSubsystems(900, pathState);
                break;
            case 13:
                follower.followPath(depositMiddle);
                arm.shoulder(Arm.Shoulder.FORWARDS);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 14:
                if (System.currentTimeMillis() - timeSnapshot < 800) {
                    arm.extendo(Arm.Extendo.RETRACTED);
                } else if (System.currentTimeMillis() - timeSnapshot > 800 && System.currentTimeMillis() - timeSnapshot < 1300) {
                    arm.extendo(Arm.Extendo.EXTENDED);
                } else {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                    arm.extendo(Arm.Extendo.EXTENDED);
                    follower.followPath(grabRight);
                    arm.intake(Arm.Intake.DEPOSIT);
                }
                break;
            case 15:
                if (System.currentTimeMillis() - timeSnapshot < 800) {
                    arm.extendo(Arm.Extendo.RETRACTED);
                } else if (System.currentTimeMillis() - timeSnapshot > 800 && System.currentTimeMillis() - timeSnapshot < 1300) {
                    arm.extendo(Arm.Extendo.EXTENDED);
                } else {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 16:
                intakeSubsystems(900, pathState);
                break;
            case 17:
                humanIntakeSubsystems(0, pathState);
                arm.intake(Arm.Intake.INTAKE);
                break;
            case 18:
                follower.followPath(humanIntakeFirst);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
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
        arm.clawRotate(Arm.ClawRotation.Horz1);
        arm.wrist(Arm.Wrist.DOWNWARDS);

        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis) {
            arm.intake(Arm.Intake.DEPOSIT);
            setPathState(pathState + 1);
            timeSnapshot = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - timeSnapshot < 300) {
            arm.extendo(Arm.Extendo.RETRACTED);
        } else {
            arm.extendo(Arm.Extendo.CHAMBER);
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

    public void humanIntakeSubsystems(int timeToWaitMilis, int pathState) {
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
        arm.shoulder(Arm.Shoulder.BACKWARDS);
        arm.extendo(Arm.Extendo.RETRACTED);
        arm.wrist(Arm.Wrist.FORWARD);
        arm.clawRotate(Arm.ClawRotation.Horz1);

        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis - 200) {
            arm.intake(Arm.Intake.CLOSE);
            if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis) {
                setPathState(pathState + 1);
                timeSnapshot = System.currentTimeMillis();
            }
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
        waitForStart();
        teamColor.initiate(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
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
            teamColor.update();
            barnacleCamera.status(telemetry);
            telemetry.addData("Heading Error", follower.headingError);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
