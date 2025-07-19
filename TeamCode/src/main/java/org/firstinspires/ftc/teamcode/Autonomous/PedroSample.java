package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.util.Constants;
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
import com.qualcomm.robotcore.hardware.Gamepad;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class PedroSample extends LinearOpMode {
    double driveErrorX = 1000;
    double driveErrorY = 1000;
    public double firstSubX = 0;
    public double firstSubY = 0;
    public double secondSubX = 0;
    public double secondSubY = 0;
    public boolean clawRotationOverideFirst = false;
    public boolean clawRotationOverideSecond = false;
    public Arm.ClawRotation clawRotationFirst = Arm.ClawRotation.Horz1;
    public Arm.ClawRotation clawRotationSecond = Arm.ClawRotation.Horz1;
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
    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));  // Starting position
    private final Pose scorePose = new Pose(9, 123, Math.toRadians(315));
    private final Pose identifyPose = new Pose(14, 115.35, Math.toRadians(360));
    private final Pose rightGrab = new Pose(14.5, 111.2, Math.toRadians(360));
    private final Pose middleGrab = new Pose(15, 120, Math.toRadians(360));
    private final Pose leftGrab = new Pose(17, 120.5, Math.toRadians(380));
    private final Pose leftDockPose = new Pose(7.5, 112, Math.toRadians(315));
    private final Pose middleDockPose = new Pose(7.5, 92, Math.toRadians(315));
    private final Pose rightDockPose = new Pose(7.5, 88, Math.toRadians(315));
    private Path scorePreload, identify, park, rightStart, right, middleStart, middle, leftStart, left, score1, score2, score3, score4, score5, scoreRight1, scoreRight2, dockLeft, dockMiddle, dockRight;
    //sub3 and sub4 are for when the barnacle is on the right
    //scoreRight 1 and 2 are for when barnacle is on the right
    private Point sub1End, sub2End, sub3End, sub4End;
    private Path sub1, sub2, sub3, sub4;
    //right1, middle2
    //right1, left2,
    //middle1, left3,
    long timeSnapshot = System.currentTimeMillis();


    public void buildPaths(double fx, double fy, double sx, double sy) {
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


        middle = new Path(new BezierLine(new Point(scorePose), new Point(middleGrab.getX(),middleGrab.getY() + 1)));
        middle.setLinearHeadingInterpolation(scorePose.getHeading(), middleGrab.getHeading());

        left = new Path(new BezierLine(new Point(scorePose), new Point(leftGrab)));
        left.setLinearHeadingInterpolation(scorePose.getHeading(), leftGrab.getHeading());

        sub1End = new Point(65.001 + fx, 99.001 + fy, Point.CARTESIAN);
        sub1 = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(62.13084112149532, 110.35514018691589, Point.CARTESIAN),
                sub1End
        )
        );
        sub1.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(270));

        sub2End = new Point(65.002 + sx, 99.002 + sy, Point.CARTESIAN);
        sub2 = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(62.13084112149532, 110.35514018691589, Point.CARTESIAN),
                sub2End
        )
        );
        sub2.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(270));

        sub3End = new Point(65.003 + fx, 99.003 + fy, Point.CARTESIAN);
        sub3 = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(13.682, 105.164, Point.CARTESIAN),
                new Point(44.411, 90.808, Point.CARTESIAN),
                new Point(67.065, 124.229, Point.CARTESIAN),
                sub3End
        )
        );
        sub3.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(270));

        sub4End = new Point(65.004 + sx, 99.004 + sy, Point.CARTESIAN);
        sub4 = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(13.682, 105.164, Point.CARTESIAN),
                new Point(44.411, 90.808, Point.CARTESIAN),
                new Point(67.065, 124.229, Point.CARTESIAN),
                sub4End
        )
        );
        sub4.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(270));


        score1 = new Path(new BezierLine(new Point(rightGrab), new Point(scorePose)));
        score1.setLinearHeadingInterpolation(rightGrab.getHeading(), scorePose.getHeading());

        score2 = new Path(new BezierLine(new Point(middleGrab), new Point(scorePose)));
        score2.setLinearHeadingInterpolation(middleGrab.getHeading(), scorePose.getHeading());

        score3 = new Path(new BezierLine(new Point(leftGrab), new Point(scorePose)));
        score3.setLinearHeadingInterpolation(leftGrab.getHeading(), scorePose.getHeading());

        score4 = new Path(
                new BezierCurve(
                        sub1End,
                        new Point(62.13084112149532, 110.35514018691589, Point.CARTESIAN),
                        new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
                )
        );
        score4.setLinearHeadingInterpolation(Math.toRadians(270), scorePose.getHeading());

        score5 = new Path(
                new BezierCurve(
                        sub2End,
                        new Point(62.13084112149532, 110.35514018691589, Point.CARTESIAN),
                        new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
                )
        );
        score5.setLinearHeadingInterpolation(Math.toRadians(270), scorePose.getHeading());

        scoreRight1 = new Path(
                new BezierCurve(
                        sub3End,
                        new Point(67.065, 124.229, Point.CARTESIAN),
                        new Point(44.411, 90.808, Point.CARTESIAN),
                        new Point(13.682, 105.164, Point.CARTESIAN),
                        new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
                )
        );
        scoreRight1.setLinearHeadingInterpolation(Math.toRadians(270), scorePose.getHeading());

        scoreRight2 = new Path(
                new BezierCurve(
                        sub4End,
                        new Point(67.065, 124.229, Point.CARTESIAN),
                        new Point(44.411, 90.808, Point.CARTESIAN),
                        new Point(13.682, 105.164, Point.CARTESIAN),
                        new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
                )
        );
        scoreRight2.setLinearHeadingInterpolation(Math.toRadians(270), scorePose.getHeading());


        dockLeft = new Path(new BezierLine(new Point(scorePose), new Point(leftDockPose)));
        dockLeft.setLinearHeadingInterpolation(scorePose.getHeading(), leftDockPose.getHeading());

        dockMiddle = new Path(new BezierLine(new Point(scorePose), new Point(middleDockPose)));
        dockMiddle.setLinearHeadingInterpolation(scorePose.getHeading(), middleDockPose.getHeading());

        dockRight = new Path(new BezierLine(new Point(scorePose), new Point(rightDockPose)));
        dockRight.setLinearHeadingInterpolation(scorePose.getHeading(), rightDockPose.getHeading());
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
                timeSnapshot = System.currentTimeMillis();
                break;
            case 3:
                restSubsystems(1100, pathState);
                break;
            case 4:
                scoutSubsystems(500, pathState);
                barnacleCamera.identifyBarnacleBucket();
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
                barnacleCamera.clear();
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
                intakeSubsystems(900, pathState);
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
                scoreSubsystems(1900, pathState);
                break;
            //Start of 4 samp
            case 16:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                    case MIDDLE:
                        follower.followPath(sub1);
                        break;
                    case RIGHT:
                        follower.followPath(sub3);
                }
                setPathState(pathState + 1);
                break;
            case 17:
                restSubsystems(1800, pathState);
                break;
            case 18:
                scoutSubsystems(1000000, pathState);
                switch (barnacleCamera.getBarnacleLocation()) {
                    case RIGHT:
                    case MIDDLE:
                        driveErrorY = Math.abs(sub3End.getY() - follower.getPose().getY());
                        driveErrorX = Math.abs(sub3End.getX() - follower.getPose().getX());
                        break;
                    case LEFT:
                        driveErrorX = Math.abs(sub1End.getX() - follower.getPose().getX());
                        driveErrorY = Math.abs(sub1End.getY() - follower.getPose().getY());
                        break;
                }
                if (driveErrorY < 3 && driveErrorX < 3) {
                    timeSnapshot = System.currentTimeMillis();
                    follower.breakFollowing();
                    setPathState(pathState + 1);
                }
                break;
            case 19:
                if (System.currentTimeMillis() - timeSnapshot > 300) {
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
                if (clawRotationOverideFirst){
                   webcam.sampleRotation = clawRotationFirst;
                }
                if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE) {
                    if (clawRotationOverideFirst){
                        arm.clawRotate(clawRotationFirst);
                    }
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 22:
                if (clawRotationOverideFirst){
                    arm.clawRotate(clawRotationFirst);
                }
                if (System.currentTimeMillis() - timeSnapshot > 300) {

                    arm.intake(Arm.Intake.CLOSE);
                }
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    switch (barnacleCamera.getBarnacleLocation()) {
                        case LEFT:
                            follower.followPath(score4);
                            break;
                        case MIDDLE:
                        case RIGHT:
                            follower.followPath(scoreRight1);
                    }
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 23:
                arm.extendo(Arm.Extendo.RETRACTED);
                arm.shoulder(Arm.Shoulder.FORWARDS);
                arm.intake(Arm.Intake.CLOSE);
                if (System.currentTimeMillis() - timeSnapshot > 300) {
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    arm.shoulder(Arm.Shoulder.UPWARDS);
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.CLOSE);
                    arm.clawRotate(Arm.ClawRotation.Horz1);

                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 24:
                if (System.currentTimeMillis() - timeSnapshot > 1500) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 25:
                scoreSubsystems(1900, pathState);
                break;
            //Start of 5 Samp
            case 26:
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(sub2);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        follower.followPath(sub4);
                }
                setPathState(pathState + 1);
                break;
            case 27:
                restSubsystems(1800, pathState);
                break;
            case 28:
                scoutSubsystems(1000000, pathState);

                switch (barnacleCamera.getBarnacleLocation()) {
                    case RIGHT:
                    case MIDDLE:
                        driveErrorY = Math.abs(sub4End.getY() - follower.getPose().getY());
                        driveErrorX = Math.abs(sub4End.getX() - follower.getPose().getX());
                        break;
                    case LEFT:
                        driveErrorX = Math.abs(sub2End.getX() - follower.getPose().getX());
                        driveErrorY = Math.abs(sub2End.getY() - follower.getPose().getY());
                        break;
                }
                if (driveErrorY < 3 && driveErrorX < 3) {
                    timeSnapshot = System.currentTimeMillis();
                    follower.breakFollowing();
                    setPathState(pathState + 1);
                }
                break;
            case 29:
                if (System.currentTimeMillis() - timeSnapshot > 300) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 30:
                webcam.setDriveStage(Webcam.DRIVE_STAGE.DRIVE);
                timeSnapshot = System.currentTimeMillis();
                setPathState(pathState + 1);
                break;
            case 31:
                if (clawRotationOverideSecond){
                    webcam.sampleRotation = clawRotationSecond;
                }
                if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE) {
                    timeSnapshot = System.currentTimeMillis();
                    if (clawRotationOverideSecond){
                        arm.clawRotate(clawRotationSecond);
                    }
                    setPathState(pathState + 1);
                }
                break;
            case 32:
                if (clawRotationOverideSecond){
                    arm.clawRotate(clawRotationSecond);
                }
                if (System.currentTimeMillis() - timeSnapshot > 300) {
                    arm.intake(Arm.Intake.CLOSE);
                }
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                    switch (barnacleCamera.getBarnacleLocation()) {
                        case LEFT:
                        case MIDDLE:
                            follower.followPath(score5);
                            break;
                        case RIGHT:
                            follower.followPath(scoreRight2);
                    }
                }
                break;
            case 33:
                arm.extendo(Arm.Extendo.RETRACTED);
                arm.shoulder(Arm.Shoulder.FORWARDS);
                arm.intake(Arm.Intake.CLOSE);
                if (System.currentTimeMillis() - timeSnapshot > 300) {
                    verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                    arm.shoulder(Arm.Shoulder.UPWARDS);
                    arm.extendo(Arm.Extendo.RETRACTED);
                    arm.wrist(Arm.Wrist.FORWARD);
                    arm.intake(Arm.Intake.CLOSE);
                    arm.clawRotate(Arm.ClawRotation.Horz1);

                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 34:
                if (System.currentTimeMillis() - timeSnapshot > 1500) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 35:
                scoreSubsystems(1900, pathState);
                break;
            case 36:
                restSubsystems(0, pathState);
                timeSnapshot = System.currentTimeMillis();
                switch (barnacleCamera.getBarnacleLocation()) {
                    case LEFT:
                        follower.followPath(dockLeft);
                        break;
                    case MIDDLE:
                        follower.followPath(dockMiddle);
                        break;
                    case RIGHT:
                        follower.followPath(dockRight);
                        break;
                }
                setPathState(pathState + 1);
            case 37:
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    arm.shoulder(Arm.Shoulder.CHAMBER_INTAKE);
                }
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
        if (System.currentTimeMillis() - timeSnapshot > 500) {
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
        } else if (System.currentTimeMillis() - timeSnapshot > 300) {
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
        barnacleCamera.initiate(hardwareMap);
        teamColor.initiate(hardwareMap);
        pathTimer = new Timer();
        constants = new Constants();
        constants.setConstants(FConstants.class, LConstants.class);
        Gamepad previousGamepad = gamepad1;
        boolean previousDown = false;
        boolean previousUp = false;
        boolean previousLeft = false;
        boolean previousRight = false;

        boolean firstSub = true;

        while (!opModeIsActive() && !isStopRequested()){
            if (gamepad1.start && firstSub){
                clawRotationOverideFirst = true;
            }
            if (gamepad1.start && !firstSub){
                clawRotationOverideSecond = true;
            }
            if (gamepad1.share && firstSub){
                clawRotationOverideFirst = false;
            }
            if (gamepad1.share && !firstSub){
                clawRotationOverideSecond = false;
            }
            if (clawRotationOverideFirst && firstSub){
                if (gamepad1.triangle){
                    clawRotationFirst = Arm.ClawRotation.Horz1;
                }
                if (gamepad1.cross){
                    clawRotationFirst = Arm.ClawRotation.Vert;
                }
                if (gamepad1.square){
                    clawRotationFirst = Arm.ClawRotation.LEFTDIAG;
                }
                if (gamepad1.circle){
                    clawRotationFirst = Arm.ClawRotation.RIGHTDIAG;
                }
            }
            if (clawRotationOverideSecond && !firstSub){
                if (gamepad1.triangle){
                    clawRotationSecond = Arm.ClawRotation.Horz1;
                }
                if (gamepad1.cross){
                    clawRotationSecond = Arm.ClawRotation.Vert;
                }
                if (gamepad1.square){
                    clawRotationSecond = Arm.ClawRotation.LEFTDIAG;
                }
                if (gamepad1.circle){
                    clawRotationSecond = Arm.ClawRotation.RIGHTDIAG;
                }
            }

            if (gamepad1.right_bumper){
                firstSub = false;
            }
            if (gamepad1.left_bumper){
                firstSub = true;
            }
            if (gamepad1.dpad_down && !previousDown){
                if (firstSub){
                    firstSubX -=1;
                }else{
                    secondSubX -=1;
                }
                previousDown = true;
            }else if (!gamepad1.dpad_down){
                previousDown = false;
            }
            if (gamepad1.dpad_up && !previousUp){
                if (firstSub){
                    firstSubX +=1;
                }else{
                    secondSubX +=1;
                }
                previousUp = true;
            }else if (!gamepad1.dpad_up){
                previousUp = false;
            }
            if (gamepad1.dpad_right && !previousRight){
                if (firstSub){
                    firstSubY -=1;
                }else{
                    secondSubY -=1;
                }
                previousRight = true;
            }else if (!gamepad1.dpad_right){
                previousRight = false;
            }
            if (gamepad1.dpad_left && !previousLeft){
                if (firstSub){
                    firstSubY +=1;
                }else{
                    secondSubY +=1;
                }
                previousLeft = true;
            }else if (!gamepad1.dpad_left){
                previousLeft = false;
            }


            previousGamepad.copy(gamepad1);
            telemetry.addLine("X IS FORWARDS (greater X = more forwards) AND Y IS SIDEWAYS (greater Y = more left) RELATIVE TO DRIVER FOR BUCKET");
            telemetry.addData("FirstX",firstSubX);
            telemetry.addData("FirstY",firstSubY);
            telemetry.addData("SecondX",secondSubX);
            telemetry.addData("SecondY",secondSubY);
            telemetry.addData("clawOverideFirst", clawRotationOverideFirst);
            telemetry.addData("clawRotFirst", clawRotationFirst);
            telemetry.addData("clawOverideSecond", clawRotationOverideSecond);
            telemetry.addData("clawRotSecond", clawRotationSecond);
            telemetry.update();
        }
        waitForStart();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths(firstSubX,firstSubY,secondSubX,secondSubY);
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
            teamColor.update();
            webcam.status(telemetry);
            barnacleCamera.status(telemetry);
            telemetry.addData("XError",driveErrorX);
            telemetry.addData("YError",driveErrorY);
            telemetry.addData("Heading Error", follower.headingError);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
