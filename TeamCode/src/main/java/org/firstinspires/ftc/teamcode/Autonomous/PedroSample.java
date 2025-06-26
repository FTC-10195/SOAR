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
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.sql.Time;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class PedroSample extends LinearOpMode {
    int pathState = 0;
    Timer pathTimer;
    Constants constants;
    Follower follower;
    TeamColor teamColor = new TeamColor(TeamColor.Color.RED);
    VerticalSlides verticalSlides;
    Arm arm = new Arm();
    DriveTrain driveTrain = new DriveTrain();
    Webcam webcam = new Webcam();
    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));  // Starting position
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));
    private final Pose barnicleIdentificationPose = new Pose(14, 129, Math.toRadians(270));
    private Path scorePreload, park, barnicleIdentification;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        barnicleIdentification = new Path(new BezierLine(new Point(scorePose), new Point(barnicleIdentificationPose)));
        barnicleIdentification.setLinearHeadingInterpolation(scorePose.getHeading(), barnicleIdentificationPose.getHeading());

    }

    public void barnacleRight() {

    }

    public void barnacleMiddle() {

    }

    public void barnacleLeft() {

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
             //   scoreSubsystems(1000);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
      /*      case 1: // Move from scoring to barnacle identification position
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
    public void scoreSubsystems(int timeToWaitMilis){
        verticalSlides.setSlidePosition(VerticalSlides.SlidePositions.BUCKET);
        arm.shoulder(Arm.Shoulder.BUCKET);
        arm.extendo(Arm.Extendo.EXTENDED);
        arm.wrist(Arm.Wrist.FORWARD);
        arm.intake(Arm.Intake.CLOSE);
        long timeSnapshot = System.currentTimeMillis();
        while (System.currentTimeMillis() - timeSnapshot < timeToWaitMilis){
            if (System.currentTimeMillis() - timeSnapshot < timeToWaitMilis){
                break;
            }
        }
        arm.intake(Arm.Intake.DEPOSIT);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;
        arm.initiate(hardwareMap);
        verticalSlides.initiate(hardwareMap);
        webcam.initiate(hardwareMap,teamColor.getColor(), StateMachine.Mode.BUCKET,telemetry);
        driveTrain.initiate(hardwareMap);
        pathTimer = new Timer();
        constants = new Constants();
        constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
