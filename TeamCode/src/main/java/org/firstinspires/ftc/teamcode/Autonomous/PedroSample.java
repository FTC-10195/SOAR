package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
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
   // BarnacleCamera barnacleCamera = new BarnacleCamera();
    Path noPath;
    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));  // Starting position
    private final Pose scorePose = new Pose(12.5, 124, Math.toRadians(315));
    private final Pose rightGrab = new Pose(14, 111, Math.toRadians(360));
    private final Pose middleGrab = new Pose(14, 121.5, Math.toRadians(360));
    private final Pose leftGrab = new Pose(14, 119, Math.toRadians(405));
    private Path scorePreload, park, right, middle, left, score1, score2, score3;
    private Path sub1;
    //right1, middle2
    //right1, left2,
    //middle1, left3,
    long timeSnapshot = System.currentTimeMillis();

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        right = new Path(new BezierLine(new Point(scorePose), new Point(rightGrab)));
        right.setLinearHeadingInterpolation(scorePose.getHeading(), rightGrab.getHeading());


        middle = new Path(new BezierLine(new Point(scorePose), new Point(middleGrab)));
        middle.setLinearHeadingInterpolation(scorePose.getHeading(), middleGrab.getHeading());

        left = new Path(new BezierLine(new Point(scorePose), new Point(leftGrab)));
        left.setLinearHeadingInterpolation(scorePose.getHeading(), leftGrab.getHeading());

        sub1 = new Path(new BezierCurve(
                new Point(12.5, 124, Point.CARTESIAN),
                new Point(63.252, 117.757, Point.CARTESIAN),
                new Point(64.000, 99.000, Point.CARTESIAN)
            )
        );
        sub1.setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()), Math.toRadians(270));


        score1 = new Path(new BezierLine(new Point(rightGrab), new Point(scorePose)));
        score1.setLinearHeadingInterpolation(rightGrab.getHeading(), scorePose.getHeading());

        score2 = new Path(new BezierLine(new Point(middleGrab), new Point(scorePose)));
        score2.setLinearHeadingInterpolation(middleGrab.getHeading(), scorePose.getHeading());

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
                restSubsystems(500,pathState);
                follower.followPath(right);
                break;
            case 3:
                scoutSubsystems(1600, pathState);
                break;
            case 4:
                intakeSubsystems(1000, pathState);
                break;
            case 5:
                restSubsystems(500, pathState);
                follower.followPath(score1);
                break;
            case 6:
                scoreSubsystems(1300, pathState);
                break;
            case 7:
            restSubsystems(500,pathState);
            follower.followPath(middle);
            break;
            case 8:
                scoutSubsystems(1600, pathState);
                break;
            case 9:
                intakeSubsystems(1000, pathState);
                break;
            case 10:
                restSubsystems(400, pathState);
                follower.followPath(score2);
                break;
            case 11:
                scoreSubsystems(1350, pathState);
                break;
            case 12:
                restSubsystems(1000, pathState);
                follower.followPath(sub1);
                break;
            case 13:
                scoutSubsystems(2000, pathState);
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
        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis) {
            setPathState(pathState + 1);
            timeSnapshot = System.currentTimeMillis();
            arm.extendo(Arm.Extendo.EXTENDED);
            arm.shoulderLerpStartTime = System.currentTimeMillis();
        } else {
            arm.extendo(Arm.Extendo.EXTENDED);
        }
    }
    public void intakeSubsystems(int timeToWaitMilis, int pathState){
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
        arm.shoulder(Arm.Shoulder.DOWNWARDS);
        arm.wrist(Arm.Wrist.DOWNWARDS);
        arm.extendo(Arm.Extendo.EXTENDED);
        if (System.currentTimeMillis() - timeSnapshot > 500 && System.currentTimeMillis() - timeSnapshot < 800) {
             arm.intake(Arm.Intake.CLOSE);
        } else if (System.currentTimeMillis() - timeSnapshot > 800) {
            arm.intake(Arm.Intake.CLOSE);
            timeSnapshot = System.currentTimeMillis();
            setPathState(pathState+1);
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
        //barnacleCamera.initiate(hardwareMap);
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
            if (webcam.currentDriveStage != Webcam.DRIVE_STAGE.DONE){
                arm.intake(webcam.intakeState);
            }
            webcam.status(telemetry);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
