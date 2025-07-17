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
    private final Pose startPose = new Pose(7, 64, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(43, 64, Math.toRadians(0));  // Starting position
    private final Pose identifyPose = new Pose(21.5,43, Math.toRadians(-38));
    private final Pose middleGrabPose = new Pose(21.5,55, Math.toRadians(-38));
    private final Pose deposit1Pose = new Pose(21.5, 45, Math.toRadians(-125));
    private final Pose deposit2Pose = new Pose(middleGrabPose.getX(),middleGrabPose.getY(), Math.toRadians(-125));
    private Path scorePreload, identify, deposit1, deposit2, intakeMiddle,intakeLeft;
    //sub3 and sub4 are for when the barnacle is on the right
    //scoreRight 1 and 2 are for when barnacle is on the right
    //right1, middle2
    //right1, left2,
    //middle1, left3,
    long timeSnapshot = System.currentTimeMillis();


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        identify = new Path(new BezierCurve(
                new Point(scorePose.getX(),scorePose.getY(),Point.CARTESIAN),
                new Point(10.409,64.483,Point.CARTESIAN),
                new Point(identifyPose.getX(),identifyPose.getY(),Point.CARTESIAN)
        ));
        identify.setLinearHeadingInterpolation(scorePose.getHeading(), identifyPose.getHeading());

        deposit1 = new Path(new BezierLine(new Point(identifyPose), new Point(deposit1Pose)));
        deposit1.setLinearHeadingInterpolation(identifyPose.getHeading(), deposit1Pose.getHeading());

        deposit2 = new Path(new BezierLine(new Point(middleGrabPose), new Point(deposit2Pose)));
        deposit2.setLinearHeadingInterpolation(middleGrabPose.getHeading(), deposit2Pose.getHeading());

    }

    public void autonomousPathUpdate() {
        double driveErrorX;
        double driveErrorY;
        switch (pathState) {
            case 0: // Move from start to scoring position
                //   scoreSubsystems(1000);
                timeSnapshot = System.currentTimeMillis();
                scoreSubsystems(100000000,pathState);
                arm.intake(Arm.Intake.CLOSE);
                setPathState(1);
                break;
            case 1:
                if (System.currentTimeMillis() - timeSnapshot > 700){
                    arm.intake(Arm.Intake.CLOSE);
                    timeSnapshot =System.currentTimeMillis();
                    setPathState(pathState+1);
                }
                break;
            case 2:
                follower.followPath(scorePreload);
                timeSnapshot =System.currentTimeMillis();
                setPathState(pathState+1);
                arm.intake(Arm.Intake.CLOSE);
              //  restSubsystems(1000,pathState);
                break;
            case 3:
                scoreSubsystems(1500,pathState);
                break;
            case 4:
                follower.followPath(identify);
                timeSnapshot =System.currentTimeMillis();
                setPathState(pathState+1);
                break;
            case 5:
                if (System.currentTimeMillis() - timeSnapshot > 500){
                    timeSnapshot =System.currentTimeMillis();
                    setPathState(pathState+1);
                }
                break;
            case 6:
                restSubsystems(1000,pathState);
                break;
            case 7:
                scoutSubsystems(800,pathState);
                arm.clawRotate(Arm.ClawRotation.Diag1);
                barnacleCamera.identifyBarnacleChamber();
                break;
            case 8:
                switch (barnacleCamera.getBarnacleLocation()){
                    case LEFT:
                        follower.followPath(intakeMiddle);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        break;
                }
                timeSnapshot =System.currentTimeMillis();
                setPathState(pathState+1);
                break;
            case 9:
                switch (barnacleCamera.getBarnacleLocation()){
                    case LEFT:
                        if (System.currentTimeMillis() - timeSnapshot > 900){
                            timeSnapshot =System.currentTimeMillis();
                            setPathState(pathState+1);
                        }
                        break;
                    case MIDDLE:
                    case RIGHT:
                        intakeSubsystems(900,pathState);
                        break;
                }
                break;
            case 10:
                switch (barnacleCamera.getBarnacleLocation()){
                    case LEFT:
                        intakeSubsystems(900,pathState);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.FORWARDS);
                        arm.wrist(Arm.Wrist.DOWNWARDS);
                        follower.followPath(deposit1);
                        timeSnapshot =System.currentTimeMillis();
                        setPathState(pathState+1);
                        break;
                }
                break;
            case 11:
                switch (barnacleCamera.getBarnacleLocation()){
                    case LEFT:
                        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.DOWN);
                        arm.shoulder(Arm.Shoulder.FORWARDS);
                        arm.wrist(Arm.Wrist.DOWNWARDS);
                        follower.followPath(deposit2);
                        timeSnapshot =System.currentTimeMillis();
                        setPathState(pathState+1);
                        break;
                    case MIDDLE:
                    case RIGHT:
                        if (System.currentTimeMillis() - timeSnapshot > 900){
                            timeSnapshot =System.currentTimeMillis();
                            arm.intake(Arm.Intake.DEPOSIT);
                            setPathState(pathState+1);
                        }
                        break;
                }
                break;
            case 12:

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

        if (System.currentTimeMillis() - timeSnapshot > timeToWaitMilis){
            arm.intake(Arm.Intake.DEPOSIT);
            setPathState(pathState+1);
            timeSnapshot = System.currentTimeMillis();
        }else if (System.currentTimeMillis() - timeSnapshot < 300){
            arm.extendo(Arm.Extendo.RETRACTED);
        }else{
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
