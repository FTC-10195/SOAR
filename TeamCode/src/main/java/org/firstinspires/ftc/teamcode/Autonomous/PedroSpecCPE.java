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
    VerticalSlides verticalSlides = new VerticalSlides();
    Arm arm = new Arm();
    DriveTrain driveTrain = new DriveTrain();
    Path noPath;
    private final Pose startPose = new Pose(7, 64, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(43, 64, Math.toRadians(0));  // Starting position
    private final Pose humanIntakePose = new Pose(10.5, 35.5, Math.toRadians(0));
    private final Pose leftGrabPose = new Pose(21.5, 43.5, Math.toRadians(-38));
    private final Pose middleGrabPose = new Pose(22, 35.5, Math.toRadians(-38));
    private final Pose rightGrabPose = new Pose(20, 23, Math.toRadians(-34));
    private final Pose depositLeftPose = new Pose(21.5, leftGrabPose.getY() + 3, Math.toRadians(-125));
    private final Pose depositMiddlePose = new Pose(21.5, middleGrabPose.getY() + 3, Math.toRadians(-125));
    private final Pose depositRightPose = new Pose(humanIntakePose.getX() + 10, rightGrabPose.getY(), Math.toRadians(0));
    private final Pose humanIntakeRightPose = new Pose(humanIntakePose.getX(), depositRightPose.getY(), Math.toRadians(0));
    private final Pose parkPose = new Pose(0, 0, Math.toRadians(90));
    private Path scorePreload, grabLeft, grabMiddle, grabRight, depositLeft, depositMiddle, depositRight1, humanIntakeFirst, scoreFirst, scoreSecond, scoreThird, scoreFourth, humanIntakeSecond, humanIntakeThird, humanIntakeFourth, park;


    long timeSnapshot = System.currentTimeMillis();


    public void buildPaths() {
        park = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY() - 2, Point.CARTESIAN),
                new Point(25, 85, Point.CARTESIAN),
                new Point(parkPose.getX(), parkPose.getY(), Point.CARTESIAN)
    ));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        park.setZeroPowerAccelerationMultiplier(4);


        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(3);

        grabLeft = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(10.409, 64.483, Point.CARTESIAN),
                new Point(leftGrabPose.getX(), leftGrabPose.getY(), Point.CARTESIAN)
        ));
        grabLeft.setLinearHeadingInterpolation(scorePose.getHeading(), leftGrabPose.getHeading());

        grabRight = new Path(new BezierCurve(new Point(depositMiddlePose.getX(), depositMiddlePose.getY(), Point.CARTESIAN),
                new Point(rightGrabPose.getX(), rightGrabPose.getY() + 5, Point.CARTESIAN),
                new Point(rightGrabPose.getX(), rightGrabPose.getY(), Point.CARTESIAN)
        )
        );
        grabRight.setLinearHeadingInterpolation(middleGrabPose.getHeading(), rightGrabPose.getHeading());

        depositRight1 = new Path(new BezierLine(new Point(rightGrabPose.getX(),rightGrabPose.getY()), new Point(depositRightPose.getX()+.001,depositRightPose.getY()+.001)));
        depositRight1.setLinearHeadingInterpolation(rightGrabPose.getHeading(), depositRightPose.getHeading());

        humanIntakeFirst = new Path(new BezierLine(new Point(depositRightPose), new Point(humanIntakeRightPose)));
        humanIntakeFirst.setLinearHeadingInterpolation(depositRightPose.getHeading(), humanIntakeRightPose.getHeading());

        depositLeft = new Path(new BezierLine(new Point(leftGrabPose), new Point(depositLeftPose)));
        depositLeft.setLinearHeadingInterpolation(leftGrabPose.getHeading(), depositLeftPose.getHeading());

        grabMiddle = new Path(new BezierLine(new Point(depositLeftPose), new Point(middleGrabPose.getX(), middleGrabPose.getY())));
        grabMiddle.setLinearHeadingInterpolation(depositLeftPose.getHeading(), middleGrabPose.getHeading());

        depositMiddle = new Path(new BezierLine(new Point(middleGrabPose), new Point(depositMiddlePose)));
        depositMiddle.setLinearHeadingInterpolation(middleGrabPose.getHeading(), depositMiddlePose.getHeading());

        scoreFirst = new Path(new BezierCurve(
                new Point(humanIntakeRightPose.getX(), humanIntakeRightPose.getY(), Point.CARTESIAN),
                new Point(18, 32, Point.CARTESIAN),
                new Point(10, 66, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN)
        ));
        scoreFirst.setLinearHeadingInterpolation(humanIntakeRightPose.getHeading(), scorePose.getHeading());


        humanIntakeSecond = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN),
                new Point(10, 71, Point.CARTESIAN),
                new Point(50, 31, Point.CARTESIAN),
                new Point(humanIntakePose.getX() - 3, humanIntakePose.getY(), Point.CARTESIAN)
        ));
        humanIntakeSecond.setLinearHeadingInterpolation(scorePose.getHeading() + Math.toRadians(15), humanIntakePose.getHeading());

        humanIntakeThird = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY() + 4, Point.CARTESIAN),
                new Point(10, 71, Point.CARTESIAN),
                new Point(50, 31, Point.CARTESIAN),
                new Point(humanIntakePose.getX() - 3, humanIntakePose.getY(), Point.CARTESIAN)
        ));
        humanIntakeThird.setLinearHeadingInterpolation(scorePose.getHeading()+ Math.toRadians(15), humanIntakePose.getHeading());
        humanIntakeFourth = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY() + 6, Point.CARTESIAN),
                new Point(10, 71, Point.CARTESIAN),
                new Point(50, 31, Point.CARTESIAN),
                new Point(humanIntakePose.getX() - 3, humanIntakePose.getY(), Point.CARTESIAN)
        ));
        humanIntakeFourth.setLinearHeadingInterpolation(scorePose.getHeading()+ Math.toRadians(15), humanIntakePose.getHeading());

        scoreSecond = new Path(new BezierCurve(
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN),
                new Point(18, 32, Point.CARTESIAN),
                new Point(7, 75, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 4, Point.CARTESIAN)
        ));
        scoreSecond.setLinearHeadingInterpolation(humanIntakePose.getHeading(), scorePose.getHeading());

        scoreThird = new Path(new BezierCurve(
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN),
                new Point(18, 32, Point.CARTESIAN),
                new Point(7, 77, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 6, Point.CARTESIAN)
        ));
        scoreThird.setLinearHeadingInterpolation(humanIntakePose.getHeading(), scorePose.getHeading());
        scoreFourth = new Path(new BezierCurve(
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN),
                new Point(18, 32, Point.CARTESIAN),
                new Point(7, 70, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() - 2, Point.CARTESIAN)
        ));
        scoreFourth.setLinearHeadingInterpolation(humanIntakePose.getHeading(), scorePose.getHeading());


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
                scoreSubsystems(1100, pathState);
                break;
            case 4:
                arm.intake(Arm.Intake.INTAKE);
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
                scoutSubsystems(1000, pathState);
                arm.clawRotate(Arm.ClawRotation.LEFTDIAG);
                break;
            case 8:
                intakeSubsystems(800, pathState);
                break;
            case 9:
                follower.followPath(depositLeft);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                arm.shoulder(Arm.Shoulder.FORWARDS);
                arm.extendo(Arm.Extendo.RETRACTED);
            case 10:
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    arm.extendo(Arm.Extendo.EXTENDED);
                } else {
                    arm.extendo(Arm.Extendo.RETRACTED);
                }
                if (System.currentTimeMillis() - timeSnapshot > 800) {
                    arm.intake(Arm.Intake.DEPOSIT);
                    follower.followPath(grabMiddle);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 11:
                if (System.currentTimeMillis() - timeSnapshot < 600 && System.currentTimeMillis() - timeSnapshot > 100) {
                    arm.extendo(Arm.Extendo.RETRACTED);
                } else {
                    arm.extendo(Arm.Extendo.EXTENDED);
                }
                if (System.currentTimeMillis() - timeSnapshot > 1400) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 12:
                intakeSubsystems(800, pathState);
                break;
            case 13:
                follower.followPath(depositMiddle);
                arm.shoulder(Arm.Shoulder.FORWARDS);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 14:
                if (System.currentTimeMillis() - timeSnapshot < 600) {
                    arm.extendo(Arm.Extendo.RETRACTED);
                } else {
                    arm.extendo(Arm.Extendo.EXTENDED);
                }

                if (System.currentTimeMillis() - timeSnapshot > 900) {
                    arm.intake(Arm.Intake.DEPOSIT);
                }
                if (System.currentTimeMillis() - timeSnapshot > 1000) {
                    arm.extendo(Arm.Extendo.RETRACTED);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                    follower.followPath(grabRight);
                }
                break;
            case 15:
                arm.shoulder(Arm.Shoulder.FORWARDS);
                if (System.currentTimeMillis() - timeSnapshot < 1100) {
                    arm.extendo(Arm.Extendo.RETRACTED);
                } else if (System.currentTimeMillis() - timeSnapshot > 1100 && System.currentTimeMillis() - timeSnapshot < 1500) {
                    arm.extendo(Arm.Extendo.EXTENDED);
                } else {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 16:
                intakeSubsystems(900, pathState);
                if (pathState != 16){
                    follower.followPath(depositRight1);
                }
                break;
            case 17:
                humanIntakeSubsystems(300, pathState);
                arm.intake(Arm.Intake.CLOSE);
                break;
            case 18:
                arm.wrist(Arm.Wrist.CHAMBER_SLIDE_DEPOSIT);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 19:
                if (System.currentTimeMillis() - timeSnapshot > 600) {
                    arm.intake(Arm.Intake.DEPOSIT);
                }
                if (System.currentTimeMillis() - timeSnapshot > 800){
                    follower.followPath(humanIntakeFirst);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 20:
                humanIntakeSubsystems(700, pathState);
                break;
            case 21:
                follower.followPath(scoreFirst);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 22:
                restSubsystems(500, pathState);
                break;
            case 23:
                scoreSubsystems(1500, pathState);
                break;
            case 24:
                follower.followPath(humanIntakeSecond);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 25:
                humanIntakeSubsystems(2300, pathState);
                if (System.currentTimeMillis() - timeSnapshot < 400){
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                }
                break;
            case 26:
                follower.followPath(scoreSecond);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 27:
                scoreSubsystems(2000, pathState);
                break;
            case 28:
                follower.followPath(humanIntakeThird);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 29:
                humanIntakeSubsystems(2200, pathState);
                if (System.currentTimeMillis() - timeSnapshot < 400){
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                }
                break;
            case 30:
                follower.followPath(scoreThird);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 31:
                scoreSubsystems(2000, pathState);
                break;
            case 32:
                follower.followPath(humanIntakeFourth);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 33:
                humanIntakeSubsystems(2200, pathState);
                if (System.currentTimeMillis() - timeSnapshot < 400){
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.CHAMBER);
                }
                break;
            case 34:
                follower.followPath(scoreFourth);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 35:
                scoreSubsystems(2000, pathState);
                break;
            case 36:
                follower.followPath(park);
                timeSnapshot = System.currentTimeMillis();
                arm.shoulder(Arm.Shoulder.UPWARDS);
                verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                arm.wrist(Arm.Wrist.FORWARD);
                setPathState(pathState + 1);
                break;
            case 37:
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
        } else if (System.currentTimeMillis() - timeSnapshot < 400) {
            arm.extendo(Arm.Extendo.RETRACTED);
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
        driveTrain.initiate(hardwareMap);
        pathTimer = new Timer();
        constants = new Constants();
        constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        waitForStart();
        follower.setStartingPose(startPose);
        buildPaths();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            verticalSlides.update();
            arm.update(telemetry, TeamColor.Color.RED);
            autonomousPathUpdate();
            follower.update();
            telemetry.addData("Heading Error", follower.headingError);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
