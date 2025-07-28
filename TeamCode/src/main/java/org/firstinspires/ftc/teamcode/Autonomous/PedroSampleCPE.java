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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.Subsystems.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class PedroSampleCPE extends LinearOpMode {
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
    TeamColor.Color teamColorFirst = TeamColor.Color.RED;
    TeamColor.Color teamColorSecond = TeamColor.Color.RED;
    VerticalSlides verticalSlides = new VerticalSlides();
    Arm arm = new Arm();
    DriveTrain driveTrain = new DriveTrain();
    Webcam webcam = new Webcam();
    private final Pose startPose = new Pose(7, 111, Math.toRadians(270));  // Starting position
    private final Pose scorePose = new Pose(9, 121.7, Math.toRadians(315));
    private final Pose rightGrabPose = new Pose(14, 112, Math.toRadians(360));
    private final Pose middleGrabPose = new Pose(14, 121, Math.toRadians(360));
    private final Pose leftGrabPose = new Pose(16, 121, Math.toRadians(380));
    private Path scorePreload, park, grabRight, grabMiddle, grabLeft, scoreRight, scoreMiddle, scoreLeft, scoreSub1, scoreSub2;
    //sub3 and sub4 are for when the barnacle is on the right
    //scoreRight 1 and 2 are for when barnacle is on the right
    private Point grabSub1Pose, grabSub2Pose;
    private Path grabSub1, grabSub2;
    long timeSnapshot = System.currentTimeMillis();
    boolean runHeadlights = false;


    public void buildPaths(double fx, double fy, double sx, double sy) {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose.getX() - 1, scorePose.getY())));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabRight = new Path(new BezierLine(new Point(scorePose), new Point(rightGrabPose)));
        grabRight.setLinearHeadingInterpolation(scorePose.getHeading(), rightGrabPose.getHeading());

        grabMiddle = new Path(new BezierLine(new Point(scorePose), new Point(middleGrabPose.getX(), middleGrabPose.getY())));
        grabMiddle.setLinearHeadingInterpolation(scorePose.getHeading(), middleGrabPose.getHeading());

        grabLeft = new Path(new BezierLine(new Point(scorePose), new Point(leftGrabPose)));
        grabLeft.setLinearHeadingInterpolation(scorePose.getHeading(), leftGrabPose.getHeading());

        grabSub1Pose = new Point(67.501 + fx, 103.001 + fy, Point.CARTESIAN);
        grabSub1 = new Path(new BezierCurve(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(62.13084112149532 + fx, 110.35514018691589 + fy, Point.CARTESIAN),
                grabSub1Pose
        )
        );
        grabSub1.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(270));
        grabSub1.setZeroPowerAccelerationMultiplier(4);

        grabSub2Pose = new Point(67.502 + sx, 103.002 + sy, Point.CARTESIAN);
        grabSub2 = new Path(new BezierCurve(
                new Point(scorePose.getX() - 2, scorePose.getY(), Point.CARTESIAN),
                new Point(62.13084112149532 + sx, 110.35514018691589 + sy, Point.CARTESIAN),
                grabSub2Pose
        )
        );
        grabSub2.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(270));
        grabSub2.setZeroPowerAccelerationMultiplier(4);

        scoreRight = new Path(new BezierLine(new Point(rightGrabPose), new Point(scorePose.getX(),scorePose.getY() + 2)));
        scoreRight.setLinearHeadingInterpolation(rightGrabPose.getHeading(), scorePose.getHeading());

        scoreMiddle = new Path(new BezierLine(new Point(middleGrabPose), new Point(scorePose.getX(),scorePose.getY()+2)));
        scoreMiddle.setLinearHeadingInterpolation(middleGrabPose.getHeading(), scorePose.getHeading());

        scoreLeft = new Path(new BezierLine(new Point(leftGrabPose.getX(), leftGrabPose.getY()), new Point(scorePose)));
        scoreLeft.setLinearHeadingInterpolation(leftGrabPose.getHeading(), scorePose.getHeading());

        scoreSub1 = new Path(
                new BezierCurve(
                        grabSub1Pose,
                        new Point(62.13084112149532 + fx, 110.35514018691589 + fy, Point.CARTESIAN),
                        new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN)
                )
        );
        scoreSub1.setLinearHeadingInterpolation(Math.toRadians(270), scorePose.getHeading());

        scoreSub2 = new Path(
                new BezierCurve(
                        grabSub2Pose,
                        new Point(62.13084112149532 + sx, 110.35514018691589 + sy, Point.CARTESIAN),
                        new Point(scorePose.getX(), scorePose.getY() + 2, Point.CARTESIAN)
                )
        );
        scoreSub2.setLinearHeadingInterpolation(Math.toRadians(270), scorePose.getHeading());
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
                arm.wrist(Arm.Wrist.FORWARD);
                scoreSubsystems(1400, pathState);
                break;
            case 2:
                if (System.currentTimeMillis() - timeSnapshot > 400) {
                    follower.followPath(grabRight);
                    setPathState(pathState + 1);
                    timeSnapshot = System.currentTimeMillis();
                }
                break;
            case 3:
                restSubsystems(1000, pathState);
                break;
            case 4:
                scoutSubsystems(800, pathState);
                break;
            case 5:
                intakeSubsystems(800, pathState);
                break;
            case 6:
                follower.followPath(scoreRight);
                setPathState(pathState + 1);
                timeSnapshot = System.currentTimeMillis();
                break;
            case 7:
                scoreSubsystems(1600, pathState);
                break;
            case 8:
                   if (System.currentTimeMillis() - timeSnapshot > 200) {
                follower.followPath(grabMiddle);
                setPathState(pathState + 1);
                timeSnapshot = System.currentTimeMillis();
                    }
                break;
            case 9:
                restSubsystems(800, pathState);
                break;
            case 10:
                scoutSubsystems(700, pathState);
                break;
            case 11:
                intakeSubsystems(800, pathState);
                break;
            case 12:
                follower.followPath(scoreMiddle);
                setPathState(pathState + 1);
                timeSnapshot = System.currentTimeMillis();
                break;
            case 13:
                scoreSubsystems(1500, pathState);
                break;
            case 14:
                if (System.currentTimeMillis() - timeSnapshot > 200) {
                    follower.followPath(grabLeft);
                    setPathState(pathState + 1);
                    timeSnapshot = System.currentTimeMillis();
                }
                break;
            case 15:
                restSubsystems(700, pathState);
                break;
            case 16:
                scoutSubsystems(700, pathState);
                break;
            case 17:
                intakeSubsystems(800, pathState);
                break;
            case 18:
                follower.followPath(scoreLeft);
                setPathState(pathState + 1);
                timeSnapshot = System.currentTimeMillis();
                break;
            case 19:
                scoreSubsystems(1700, pathState);
                break;
            case 20:
                if (System.currentTimeMillis() - timeSnapshot > 200) {
                    follower.followPath(grabSub1);
                    setPathState(pathState + 1);
                    timeSnapshot = System.currentTimeMillis();
                }
                break;
            case 21:
                restSubsystems(800, pathState);
                break;
            case 22:
                scoutSubsystems(1000, pathState);
                break;
            case 23:
                driveErrorX = Math.abs(follower.getPose().getX() - grabSub1Pose.getX());
                driveErrorY = Math.abs(follower.getPose().getY() - grabSub1Pose.getY());
                if (driveErrorX < 2 && driveErrorY < 2) {
                    follower.breakFollowing();
                    setPathState(pathState + 1);
                    runHeadlights = true;
                    timeSnapshot = System.currentTimeMillis();
                }
                break;
            case 24:
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    webcam.setDriveStage(Webcam.DRIVE_STAGE.DRIVE);
                    setPathState(pathState + 1);
                    timeSnapshot = System.currentTimeMillis();
                }
                break;
            case 25:
                if (clawRotationOverideFirst) {
                    webcam.sampleRotation = clawRotationFirst;
                }
                if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE) {
                    if (clawRotationOverideFirst) {
                        arm.clawRotate(clawRotationFirst);
                        webcam.sampleRotation = clawRotationFirst;
                    }
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 26:
                if (clawRotationOverideFirst) {
                    webcam.sampleRotation = clawRotationFirst;
                    arm.clawRotate(clawRotationFirst);
                }
                if (System.currentTimeMillis() - timeSnapshot < 200) {
                    arm.intake(Arm.Intake.CLOSE);
                } else {
                    follower.followPath(scoreSub1);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 27:
                arm.shoulder(Arm.Shoulder.FORWARDS);
                if (System.currentTimeMillis() - timeSnapshot > 400) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 28:
                restSubsystems(1000, pathState);
                break;
            case 29:
                scoreSubsystems(1400, pathState);
                break;
            case 30:
                if (System.currentTimeMillis() - timeSnapshot > 200) {
                    webcam.setColorLocatorTeam(teamColorSecond, true);
                    teamColor.setColor(teamColorSecond);
                    follower.followPath(grabSub2);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 31:
                restSubsystems(800, pathState);
                break;
            case 32:
                scoutSubsystems(1000, pathState);
                break;
            case 33:
                driveErrorX = Math.abs(follower.getPose().getX() - grabSub2Pose.getX());
                driveErrorY = Math.abs(follower.getPose().getY() - grabSub2Pose.getY());
                if (driveErrorX < 2 && driveErrorY < 2) {
                    follower.breakFollowing();
                    setPathState(pathState + 1);
                    timeSnapshot = System.currentTimeMillis();
                }
                break;
            case 34:
                if (System.currentTimeMillis() - timeSnapshot > 400) {
                    webcam.setDriveStage(Webcam.DRIVE_STAGE.DRIVE);
                    setPathState(pathState + 1);
                    timeSnapshot = System.currentTimeMillis();
                }
                break;
            case 35:
                if (clawRotationOverideSecond) {
                    arm.clawRotate(clawRotationSecond);
                    webcam.sampleRotation = clawRotationSecond;
                }
                if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DONE) {
                    if (clawRotationOverideSecond) {
                        arm.clawRotate(clawRotationSecond);
                        webcam.sampleRotation = clawRotationSecond;
                    }
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 36:
                if (clawRotationOverideSecond) {
                    arm.clawRotate(clawRotationSecond);
                    webcam.sampleRotation = clawRotationSecond;
                }
                if (System.currentTimeMillis() - timeSnapshot < 200) {
                    arm.intake(Arm.Intake.CLOSE);
                } else {
                    follower.followPath(scoreSub2);
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 37:
                arm.shoulder(Arm.Shoulder.FORWARDS);
                if (System.currentTimeMillis() - timeSnapshot > 500) {
                    timeSnapshot = System.currentTimeMillis();
                    setPathState(pathState + 1);
                }
                break;
            case 38:
                restSubsystems(1000, pathState);
                break;
            case 39:
                scoreSubsystems(1500, pathState);
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
        if (System.currentTimeMillis() - timeSnapshot > 600) {
            arm.extendo(Arm.Extendo.BUCKET);
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
        boolean triggerPressed = false;
        while (!opModeIsActive() && !isStopRequested()) {
            teamColor.update();
            if (gamepad1.start && firstSub) {
                clawRotationOverideFirst = true;
            }
            if (gamepad1.start && !firstSub) {
                clawRotationOverideSecond = true;
            }
            if (gamepad1.share && firstSub) {
                clawRotationOverideFirst = false;
            }
            if (gamepad1.share && !firstSub) {
                clawRotationOverideSecond = false;
            }
            if (clawRotationOverideFirst && firstSub) {
                if (gamepad1.triangle) {
                    clawRotationFirst = Arm.ClawRotation.Horz1;
                }
                if (gamepad1.cross) {
                    clawRotationFirst = Arm.ClawRotation.Vert;
                }
                if (gamepad1.square) {
                    clawRotationFirst = Arm.ClawRotation.LEFTDIAG;
                }
                if (gamepad1.circle) {
                    clawRotationFirst = Arm.ClawRotation.RIGHTDIAG;
                }
            }
            if (clawRotationOverideSecond && !firstSub) {
                if (gamepad1.triangle) {
                    clawRotationSecond = Arm.ClawRotation.Horz1;
                }
                if (gamepad1.cross) {
                    clawRotationSecond = Arm.ClawRotation.Vert;
                }
                if (gamepad1.square) {
                    clawRotationSecond = Arm.ClawRotation.LEFTDIAG;
                }
                if (gamepad1.circle) {
                    clawRotationSecond = Arm.ClawRotation.RIGHTDIAG;
                }
            }

            if (gamepad1.right_bumper) {
                teamColor.setColor(teamColorSecond);
                firstSub = false;
            }
            if (gamepad1.left_bumper) {
                teamColor.setColor(teamColorFirst);
                firstSub = true;
            }
            if (gamepad1.dpad_down && !previousDown) {
                if (firstSub) {
                    firstSubX -= 1;
                } else {
                    secondSubX -= 1;
                }
                previousDown = true;
            } else if (!gamepad1.dpad_down) {
                previousDown = false;
            }
            if (gamepad1.dpad_up && !previousUp) {
                if (firstSub) {
                    firstSubX += 1;
                } else {
                    secondSubX += 1;
                }
                previousUp = true;
            } else if (!gamepad1.dpad_up) {
                previousUp = false;
            }
            if (gamepad1.dpad_right && !previousRight) {
                if (firstSub) {
                    firstSubY -= 1;
                } else {
                    secondSubY -= 1;
                }
                previousRight = true;
            } else if (!gamepad1.dpad_right) {
                previousRight = false;
            }
            if (gamepad1.dpad_left && !previousLeft) {
                if (firstSub) {
                    firstSubY += 1;
                } else {
                    secondSubY += 1;
                }
                previousLeft = true;
            } else if (!gamepad1.dpad_left) {
                previousLeft = false;
            }

            if (gamepad1.right_trigger > 0.1 && !triggerPressed) {
                if (firstSub) {
                    switch (teamColorFirst) {
                        case RED:
                            teamColorFirst = (TeamColor.Color.BLUE);
                            break;
                        case BLUE:
                            teamColorFirst = (TeamColor.Color.WHITE);
                            break;
                        case WHITE:
                            teamColorFirst = (TeamColor.Color.YELLOW);
                            break;
                        case YELLOW:
                            teamColorFirst = (TeamColor.Color.NONE);
                            break;
                        case BOTH:
                        case NONE:
                            teamColorFirst = (TeamColor.Color.RED);
                            break;
                    }

                } else {
                    switch (teamColorSecond) {
                        case RED:
                            teamColorSecond = (TeamColor.Color.BLUE);
                            break;
                        case BLUE:
                            teamColorSecond = (TeamColor.Color.WHITE);
                            break;
                        case WHITE:
                            teamColorSecond = (TeamColor.Color.YELLOW);
                            break;
                        case YELLOW:
                            teamColorSecond = (TeamColor.Color.NONE);
                            break;
                        case BOTH:
                        case NONE:
                            teamColorSecond = (TeamColor.Color.RED);
                            break;
                    }
                }
                triggerPressed = true;
            } else if (gamepad1.right_trigger < .1) {
                triggerPressed = false;
            }

            previousGamepad.copy(gamepad1);
            telemetry.addLine("X IS FORWARDS (greater X = more forwards) AND Y IS SIDEWAYS (greater Y = more left) RELATIVE TO DRIVER FOR BUCKET");
            telemetry.addData("FirstX", firstSubX);
            telemetry.addData("FirstY", firstSubY);
            telemetry.addData("SecondX", secondSubX);
            telemetry.addData("SecondY", secondSubY);
            telemetry.addData("clawOverideFirst", clawRotationOverideFirst);
            telemetry.addData("clawRotFirst", clawRotationFirst);
            telemetry.addData("clawOverideSecond", clawRotationOverideSecond);
            telemetry.addData("clawRotSecond", clawRotationSecond);
            telemetry.addData("teamColor1", teamColorFirst);
            telemetry.addData("teamColor2", teamColorSecond);
            telemetry.update();
        }
        waitForStart();
        teamColor.setColor(teamColorFirst);
        webcam.setColorLocatorTeam(teamColorFirst, true);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths(firstSubX, firstSubY, secondSubX, secondSubY);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            teamColor.runHeadlights(runHeadlights);
            verticalSlides.update();
            arm.update(telemetry, teamColor.getColor());
            autonomousPathUpdate();
            follower.update();
            if (webcam.currentDriveStage != Webcam.DRIVE_STAGE.DONE) {
                TelemetryPacket packet = new TelemetryPacket();
                // arm.intake(webcam.intakeState);
                webcam.update(driveTrain, arm, packet);
            }
            if (webcam.currentDriveStage == Webcam.DRIVE_STAGE.DROP) {
                arm.clawRotate(webcam.sampleRotation);
            }
            teamColor.update();
            webcam.status(telemetry);
            telemetry.addData("XError", driveErrorX);
            telemetry.addData("YError", driveErrorY);
            telemetry.addData("Heading Error", follower.headingError);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
