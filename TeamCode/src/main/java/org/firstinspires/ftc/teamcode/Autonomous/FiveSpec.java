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
public class FiveSpec extends LinearOpMode {
    public double firstSubX = 0;
    public double firstSubY = 0;
    public boolean clawRotationOveride = false;
    public Arm.ClawRotation clawRotation = Arm.ClawRotation.Horz1;
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
    private final Pose startPose = new Pose(7, 64, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(43, 64, Math.toRadians(0));
    private final Pose intakeSubPose = new Pose(41, 64, Math.toRadians(0));
    private final Pose depositSubPose = new Pose(21, 48.7, Math.toRadians(-120));
    private final Pose humanIntakePose = new Pose(11, 35, Math.toRadians(0));
    private final Pose identifyPose = new Pose(21.5, 42.7, Math.toRadians(-38));
    private final Pose middleGrabPose = new Pose(22, 35, Math.toRadians(-38));
    private final Pose rightGrabPose = new Pose(22, 25.7, Math.toRadians(-38));
    private final Pose depositFirstLeftPose = new Pose(21.5, identifyPose.getY() + 3, Math.toRadians(-125));
    private final Pose depositFirstMiddlePose = new Pose(middleGrabPose.getX(), middleGrabPose.getY() + 2, Math.toRadians(-125));
    private final Pose depositSecondMiddlePose = new Pose(humanIntakePose.getX() + 6, middleGrabPose.getY() - 3, Math.toRadians(0));
    private final Pose depositSecondRightPose = new Pose(humanIntakePose.getX() + 6, rightGrabPose.getY() - 3, Math.toRadians(0));
    private final Pose dockLeftPose = new Pose(15, 125, Math.toRadians(0));
    private final Pose dockMiddlePose = new Pose(15, 105, Math.toRadians(0));
    private final Pose dockRightPose = new Pose(15, 80, Math.toRadians(0));
    private final Pose humanIntakeMiddlePose = new Pose(humanIntakePose.getX(), depositSecondMiddlePose.getY(), Math.toRadians(0));
    private final Pose humanIntakeRightPose = new Pose(humanIntakePose.getX(), depositSecondRightPose.getY(), Math.toRadians(0));
    private Path scorePreload, intakeSub, depositSub, identify, firstGrabMiddle, secondGrabMiddle, secondGrabRightPreviousLeft, secondGrabRightPreviousMiddle, depositFirstLeft, depositFirstMiddle, depositSecondMiddle, depositSecondRight, humanIntakeMiddle, humanIntakeRight, scoreFirstMiddle, scoreFirstRight, scoreSecond, scoreThird, scoreFourth, humanIntakeSecond, humanIntakeThird, humanIntakeFourth, dockLeft, dockMiddle, dockRight;


    long timeSnapshot = System.currentTimeMillis();


    public void buildPaths(double fx, double fy) {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(3);

        intakeSub = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(10, 64, Point.CARTESIAN),
                new Point(intakeSubPose.getX() +fx, intakeSubPose.getY() +fy, Point.CARTESIAN)
        ));
        intakeSub.setLinearHeadingInterpolation(scorePose.getHeading(), intakeSubPose.getHeading());

        depositSub = new Path(new BezierCurve(
                new Point(intakeSubPose.getX() +fx, intakeSubPose.getY() +fy, Point.CARTESIAN),
                new Point(19, 100, Point.CARTESIAN),
                new Point(depositSubPose.getX(), depositSubPose.getY(), Point.CARTESIAN)
        ));
        depositSub.setLinearHeadingInterpolation(intakeSubPose.getHeading(), depositSubPose.getHeading());

        identify = new Path(new BezierLine(
                new Point(depositSubPose),
                new Point(identifyPose)
        ));
        identify.setLinearHeadingInterpolation(depositSubPose.getHeading(), identifyPose.getHeading());


        //BARNACLE LEFT
        //firstGrabMiddle -> depositFirstMiddle -> secondGrabRightPreviousMiddle -> depositSecondMiddle -> scoring

        firstGrabMiddle = new Path(new BezierLine(new Point(identifyPose), new Point(middleGrabPose.getX(), middleGrabPose.getY() - .7)));
        firstGrabMiddle.setLinearHeadingInterpolation(identifyPose.getHeading(), middleGrabPose.getHeading());

        depositFirstMiddle = new Path(new BezierLine(new Point(middleGrabPose), new Point(depositFirstMiddlePose)));
        depositFirstMiddle.setLinearHeadingInterpolation(middleGrabPose.getHeading(), depositFirstMiddlePose.getHeading());

        secondGrabRightPreviousMiddle = new Path(new BezierLine(new Point(depositFirstMiddlePose), new Point(rightGrabPose)));
        secondGrabRightPreviousMiddle.setLinearHeadingInterpolation(depositFirstMiddlePose.getHeading(), rightGrabPose.getHeading());

        depositSecondRight = new Path(new BezierLine(new Point(rightGrabPose), new Point(depositSecondRightPose)));
        depositSecondRight.setLinearHeadingInterpolation(rightGrabPose.getHeading(), depositSecondRightPose.getHeading());

        humanIntakeRight = new Path(new BezierLine(new Point(depositSecondRightPose), new Point(humanIntakeRightPose)));
        humanIntakeRight.setLinearHeadingInterpolation(depositSecondRightPose.getHeading(), humanIntakeRightPose.getHeading());

        //BARNACLE MIDDLE
        //identify -> depositFirstLeft -> secondGrabRightPreviousLeft -> depositSecondRight -> scoring

        depositFirstLeft = new Path(new BezierLine(new Point(identifyPose), new Point(depositFirstLeftPose)));
        depositFirstLeft.setLinearHeadingInterpolation(identifyPose.getHeading(), depositFirstLeftPose.getHeading());

        secondGrabRightPreviousLeft = new Path(new BezierLine(new Point(depositFirstLeftPose), new Point(rightGrabPose)));
        secondGrabRightPreviousLeft.setLinearHeadingInterpolation(depositFirstLeftPose.getHeading(), rightGrabPose.getHeading());


        //BARNACLE RIGHT
        //identify -> depositFirst1 -> secondGrabMiddle -> depositSecond1 -> scoring

        secondGrabMiddle = new Path(new BezierLine(new Point(depositFirstLeftPose), new Point(middleGrabPose)));
        secondGrabMiddle.setLinearHeadingInterpolation(depositFirstLeftPose.getHeading(), middleGrabPose.getHeading());

        depositSecondMiddle = new Path(new BezierLine(new Point(middleGrabPose), new Point(depositSecondMiddlePose)));
        depositSecondMiddle.setLinearHeadingInterpolation(middleGrabPose.getHeading(), depositSecondMiddlePose.getHeading());

        humanIntakeMiddle = new Path(new BezierLine(new Point(depositSecondMiddlePose), new Point(humanIntakeMiddlePose)));
        humanIntakeMiddle.setLinearHeadingInterpolation(depositSecondMiddlePose.getHeading(), humanIntakeMiddlePose.getHeading());

        scoreFirstMiddle = new Path(new BezierCurve(
                new Point(humanIntakeMiddlePose.getX(), humanIntakeMiddlePose.getY(), Point.CARTESIAN),
                new Point(30, 33, Point.CARTESIAN),
                new Point(17, 64, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN)
        ));
        scoreFirstMiddle.setLinearHeadingInterpolation(humanIntakeMiddlePose.getHeading(), scorePose.getHeading());

        scoreFirstRight = new Path(new BezierCurve(
                new Point(humanIntakeRightPose.getX(), humanIntakeRightPose.getY(), Point.CARTESIAN),
                new Point(30, 33, Point.CARTESIAN),
                new Point(17, 64, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN)
        ));
        scoreFirstRight.setLinearHeadingInterpolation(humanIntakeRightPose.getHeading(), scorePose.getHeading());


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

        humanIntakeFourth = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY() + 6, Point.CARTESIAN),
                new Point(26, 64, Point.CARTESIAN),
                new Point(40, 33, Point.CARTESIAN),
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN)
        ));
        humanIntakeFourth.setLinearHeadingInterpolation(scorePose.getHeading(), humanIntakePose.getHeading());

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
        scoreFourth = new Path(new BezierCurve(
                new Point(humanIntakePose.getX(), humanIntakePose.getY(), Point.CARTESIAN),
                new Point(22, 62, Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY() - 2, Point.CARTESIAN)
        ));
        scoreFourth.setLinearHeadingInterpolation(humanIntakePose.getHeading(), scorePose.getHeading());

        dockLeft = new Path(new BezierLine(new Point(scorePose.getX(), scorePose.getY() - 2), new Point(dockLeftPose)));
        dockLeft.setLinearHeadingInterpolation(scorePose.getHeading(), dockLeftPose.getHeading());

        dockRight = new Path(new BezierLine(new Point(scorePose.getX(), scorePose.getY() - 2), new Point(dockRightPose)));
        dockRight.setLinearHeadingInterpolation(scorePose.getHeading(), dockRightPose.getHeading());

        dockMiddle = new Path(new BezierLine(new Point(scorePose.getX(), scorePose.getY() - 2), new Point(dockMiddlePose)));
        dockMiddle.setLinearHeadingInterpolation(scorePose.getHeading(), dockMiddlePose.getHeading());

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
                follower.followPath(intakeSub);
                arm.shoulder(Arm.Shoulder.BUCKET);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 5:
                if (System.currentTimeMillis() - timeSnapshot > 900) {
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.extendo(Arm.Extendo.EXTENDED);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                } else if (System.currentTimeMillis() - timeSnapshot > 500) {
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    arm.shoulder(Arm.Shoulder.FORWARDS);
                    arm.wrist(Arm.Wrist.DOWNWARDS);
                    arm.extendo(Arm.Extendo.RETRACTED);
                }
                if (System.currentTimeMillis() - timeSnapshot > 1300) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                    follower.breakFollowing();
                }
                break;
            case 6:
                webcam.snapshot();
                webcam.setDriveStage(Webcam.DRIVE_STAGE.DRIVE);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 7:
                if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 8:
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    arm.intake(Arm.Intake.CLOSE);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 9:
                arm.shoulder(Arm.Shoulder.FORWARDS);
                arm.extendo(Arm.Extendo.RETRACTED);
                follower.followPath(depositSub);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 10:
                if (System.currentTimeMillis() - timeSnapshot > 1000) {
                    arm.extendo(Arm.Extendo.EXTENDED);
                }
                if (System.currentTimeMillis() - timeSnapshot > 2000) {
                    arm.intake(Arm.Intake.DEPOSIT);
                    follower.followPath(identify);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 11:
                restSubsystems(1000, pathState);
                break;
            case 12:
                restSubsystems(1000, pathState);
                break;
            case 13:
                scoutSubsystems(800, pathState);
                arm.clawRotate(Arm.ClawRotation.LEFTDIAG);
                barnacleCamera.identifyBarnacleChamber();
                break;
            case 14:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(firstGrabMiddle);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        break;
                }
                barnacleCamera.clear();
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 15:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        if (System.currentTimeMillis() - timeSnapshot > 1200) {
                            timeSnapshot = System.currentTimeMillis();
                            setPathState(pathState + 1);
                        }
                        break;
                    case MIDDLE:
                    case RIGHT:
                        //Sample 1
                        intakeSubsystems(900, pathState);
                        break;
                }
                break;
            case 16:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        intakeSubsystems(900, pathState);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.FORWARDS);
                        arm.wrist(Arm.Wrist.DOWNWARDS);
                        follower.followPath(depositFirstLeft);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 17:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.FORWARDS);
                        arm.wrist(Arm.Wrist.DOWNWARDS);
                        follower.followPath(depositFirstMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        if (System.currentTimeMillis() - timeSnapshot > 900) {
                            timeSnapshot = System.currentTimeMillis();
                            arm.intake(Arm.Intake.DEPOSIT);
                            setPathState(pathState + 1);
                        }
                        break;
                }
                break;
            case 18:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        if (System.currentTimeMillis() - timeSnapshot < 500) {
                            arm.extendo(Arm.Extendo.RETRACTED);
                        } else {
                            arm.extendo(Arm.Extendo.EXTENDED);
                        }
                        if (System.currentTimeMillis() - timeSnapshot > 900) {
                            timeSnapshot = System.currentTimeMillis();
                            arm.intake(Arm.Intake.DEPOSIT);
                            setPathState(pathState + 1);
                        }
                        break;
                    case RIGHT:
                        follower.followPath(secondGrabMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case MIDDLE:
                        follower.followPath(secondGrabRightPreviousLeft);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 19:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(secondGrabRightPreviousMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                        scoutSubsystems(1300, pathState);
                        break;
                    case MIDDLE:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.FORWARDS);
                        arm.wrist(Arm.Wrist.DOWNWARDS);
                        arm.intake(Arm.Intake.INTAKE);
                        if (System.currentTimeMillis() - timeSnapshot > 1300) {
                            setPathState(pathState + 1);
                            timeSnapshot = System.currentTimeMillis();
                            arm.extendo(Arm.Extendo.EXTENDED);
                            arm.shoulderLerpStartTime = System.currentTimeMillis();
                        } else if (System.currentTimeMillis() - timeSnapshot < 900) {
                            arm.extendo(Arm.Extendo.RETRACTED);
                        } else {
                            arm.extendo(Arm.Extendo.EXTENDED);
                        }
                        break;
                }
                break;
            case 20:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.FORWARDS);
                        arm.wrist(Arm.Wrist.DOWNWARDS);
                        arm.intake(Arm.Intake.INTAKE);
                        if (System.currentTimeMillis() - timeSnapshot > 1300) {
                            setPathState(pathState + 1);
                            timeSnapshot = System.currentTimeMillis();
                            arm.extendo(Arm.Extendo.EXTENDED);
                            arm.shoulderLerpStartTime = System.currentTimeMillis();
                        } else if (System.currentTimeMillis() - timeSnapshot < 900) {
                            arm.extendo(Arm.Extendo.RETRACTED);
                        } else {
                            arm.extendo(Arm.Extendo.EXTENDED);
                        }
                        break;
                    case RIGHT:
                    case MIDDLE:
                        intakeSubsystems(900, pathState);
                        break;
                }
                break;
            case 21:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        intakeSubsystems(900, pathState);
                        break;
                    case RIGHT:
                        follower.followPath(depositSecondMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case MIDDLE:
                        follower.followPath(depositSecondRight);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 22:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(depositSecondRight);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.BACKWARDS);
                        arm.extendo(Arm.Extendo.RETRACTED);
                        arm.wrist(Arm.Wrist.FORWARD);
                        arm.clawRotate(Arm.ClawRotation.Horz1);
                        if (System.currentTimeMillis() - timeSnapshot > 900) {
                            arm.intake(Arm.Intake.DEPOSIT);
                            timeSnapshot = System.currentTimeMillis();
                            setPathState(pathState + 1);
                        }
                        break;
                }
                break;
            case 23:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.BACKWARDS);
                        arm.extendo(Arm.Extendo.RETRACTED);
                        arm.wrist(Arm.Wrist.FORWARD);
                        arm.clawRotate(Arm.ClawRotation.Horz1);
                        if (System.currentTimeMillis() - timeSnapshot > 900) {
                            arm.intake(Arm.Intake.DEPOSIT);
                            timeSnapshot = System.currentTimeMillis();
                            setPathState(pathState + 1);
                        }
                        break;
                    case RIGHT:
                    case MIDDLE:
                        if (System.currentTimeMillis() - timeSnapshot > 300) {
                            arm.intake(Arm.Intake.DEPOSIT);
                            if (System.currentTimeMillis() - timeSnapshot > 600) {
                                timeSnapshot = System.currentTimeMillis();
                                setPathState(pathState + 1);
                            }
                        }
                }
                break;
            case 24:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        if (System.currentTimeMillis() - timeSnapshot > 300) {
                            arm.intake(Arm.Intake.DEPOSIT);
                            if (System.currentTimeMillis() - timeSnapshot > 600) {
                                timeSnapshot = System.currentTimeMillis();
                                setPathState(pathState + 1);
                            }
                        }
                        break;
                    case RIGHT:
                        follower.followPath(humanIntakeMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case MIDDLE:
                        follower.followPath(humanIntakeRight);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 25:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(humanIntakeRight);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        if (System.currentTimeMillis() - timeSnapshot > 500) {
                            arm.intake(Arm.Intake.CLOSE);
                            if (System.currentTimeMillis() - timeSnapshot > 800) {
                                timeSnapshot = System.currentTimeMillis();
                                setPathState(pathState + 1);
                            }
                        }
                        break;
                }
                break;
            case 26:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        if (System.currentTimeMillis() - timeSnapshot > 500) {
                            arm.intake(Arm.Intake.CLOSE);
                            if (System.currentTimeMillis() - timeSnapshot > 800) {
                                timeSnapshot = System.currentTimeMillis();
                                setPathState(pathState + 1);
                            }
                        }
                        break;
                    case RIGHT:
                        follower.followPath(scoreFirstMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case MIDDLE:
                        follower.followPath(scoreFirstRight);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 27:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(scoreFirstMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        scoreSubsystems(1900, pathState);
                        break;
                }
                break;
            case 28:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        scoreSubsystems(1900, pathState);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        follower.followPath(humanIntakeSecond);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 29:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(humanIntakeSecond);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        humanIntakeSubsystems(2500, pathState);
                        break;
                }
                break;
            case 30:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        humanIntakeSubsystems(2500, pathState);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        follower.followPath(scoreSecond);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 31:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(scoreSecond);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        scoreSubsystems(1900, pathState);
                        break;
                }
                break;
            case 32:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        scoreSubsystems(1900, pathState);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        follower.followPath(humanIntakeThird);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 33:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(humanIntakeThird);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        humanIntakeSubsystems(2500, pathState);
                        break;
                }
                break;
            case 34:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        humanIntakeSubsystems(2500, pathState);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        follower.followPath(scoreThird);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 35:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(scoreThird);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        scoreSubsystems(1900, pathState);
                        break;
                }
                break;


            case 36:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        scoreSubsystems(1900, pathState);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        follower.followPath(humanIntakeFourth);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 37:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(humanIntakeFourth);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        humanIntakeSubsystems(2500, pathState);
                        break;
                }
                break;
            case 38:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        humanIntakeSubsystems(2500, pathState);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        follower.followPath(scoreFourth);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 39:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(scoreFourth);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                    case MIDDLE:
                        scoreSubsystems(2000, pathState);
                        break;
                }
                break;


            case 40:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        scoreSubsystems(2000, pathState);
                        break;
                    case RIGHT:
                        follower.followPath(dockRight);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case MIDDLE:
                        follower.followPath(dockMiddle);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                }
                break;
            case 41:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(dockLeft);
                        timeSnapshot = System.currentTimeMillis();
                        setPathState(pathState + 1);
                        break;
                    case RIGHT:
                        humanIntakeSubsystems(0, pathState);
                        break;
                    case MIDDLE:
                        humanIntakeSubsystems(0, pathState);
                        break;
                }
                break;
            case 42:
                humanIntakeSubsystems(0, pathState);
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
        webcam.initiate(hardwareMap, teamColor.getColor(), StateMachine.Mode.CHAMBER, telemetry);
        driveTrain.initiate(hardwareMap);
        barnacleCamera.initiate(hardwareMap);
        pathTimer = new Timer();
        constants = new Constants();
        constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        boolean previousDown = false;
        boolean previousUp = false;
        boolean previousLeft = false;
        boolean previousRight = false;
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.start) {
                clawRotationOveride = true;
            }
            if (gamepad1.share) {
                clawRotationOveride = false;
            }
            if (clawRotationOveride) {
                if (gamepad1.triangle) {
                    clawRotation = Arm.ClawRotation.Horz1;
                }
                if (gamepad1.cross) {
                    clawRotation = Arm.ClawRotation.Vert;
                }
                if (gamepad1.square) {
                    clawRotation = Arm.ClawRotation.LEFTDIAG;
                }
                if (gamepad1.circle) {
                    clawRotation = Arm.ClawRotation.RIGHTDIAG;
                }
            }
            if (gamepad1.dpad_down && !previousDown) {
                firstSubX -= 1;
                previousDown = true;
            } else if (!gamepad1.dpad_down) {
                previousDown = false;
            }
            if (gamepad1.dpad_up && !previousUp) {
                firstSubX += 1;
                previousUp = true;
            } else if (!gamepad1.dpad_up) {
                previousUp = false;
            }
            if (gamepad1.dpad_right && !previousRight) {
                firstSubY += 1;
                previousRight = true;
            } else if (!gamepad1.dpad_right) {
                previousRight = false;
            }
            if (gamepad1.dpad_left && !previousLeft) {
                firstSubY -= 1;
                previousLeft = true;
            } else if (!gamepad1.dpad_left) {
                previousLeft = false;
            }
        }
        waitForStart();
        teamColor.initiate(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths(firstSubX,firstSubY);
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
