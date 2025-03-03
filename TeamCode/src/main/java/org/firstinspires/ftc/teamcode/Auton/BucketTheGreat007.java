package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Config
@Autonomous(name = "007 Bucket")
public class BucketTheGreat007 extends OpMode {
    Servo intakeBarL; //0C
    // right Servo
    Servo intakeBarR; //0E
    // moves entire claw up and down
    Servo intakeElbow; //1E
    // moves outtake claw up and down
    Servo shoulderR; //2C
    Servo shoulderL; //2E
    // turns claw left and right
    Servo intakeWrist; //4E
    // opens and closes claw
    Servo intakeClaw; //5E
    Servo outtakeElbow; //1C
    Servo outtakeWrist;//4C
    Servo outtakeClaw; //5C

    // moves vertical slides up and down
    DcMotor slideMotorR; //3E
    DcMotor slideMotorL; //3C

    //Time variables:
    public static int bucketWristWait = 900;
    public static int bucketClawWait = 400;
    public static int bucketRetractWait = 400;
    public static int bucketDownWait = 400;
    public static int clawCloseWait = 1400;
    public static int clawCloseWaitLast = 1800;

    public static double slurpLowerBound = 0.21;
    public static double slurpUpperBound = 0.65;

    //Pose variables

    public static double sample3x = 18.4;
    public static double sample3y = 6.7;
    public static double sample3heading = 56.7;

    public static double sample2x = 9.98;
    public static double sample2y = 19;
    public static double sample2heading = 0;

    public static double sample1x = 9.98;
    public static double sample1y = 10.76;
    public static double sample1heading = 0;

    public static double bucketX = 5.3;
    public static double bucketY = 17;
    public static double bucketHeading = 315;

    public static int bucketSlidePos = 850;

    public boolean bucketFinished = false;

    public String bucketState = "init";
    public String pathState = "init";
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose sample1 = new Pose(sample1x, sample1y, Math.toRadians(sample1heading));
    private final Pose sample2 = new Pose(sample2x, sample2y, Math.toRadians(sample2heading)); //ywas18.5
    private final Pose sample3 = new Pose(sample3x, sample3y, Math.toRadians(sample3heading));
    private final Pose bucketPose = new Pose(bucketX, bucketY, Math.toRadians(bucketHeading));

    private Follower follower;
    private PathChain bucketDrop, sample1Snag, sample2Snag, sample3Snag;
    long timePassed;
    Timer opmodeTimer;
    Timer pathTimer;
    Timer bucketTimer;

    @Override
    public void init() {
        intakeBarL = hardwareMap.get(Servo.class, "intakeBarL");
        intakeBarL.scaleRange(0.35, 0.67);
        intakeBarL.setPosition(1);
        intakeBarR = hardwareMap.get(Servo.class, "intakeBarR");
        intakeBarR.scaleRange(0.33, 0.65);
        intakeBarR.setPosition(0);

        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeClaw.setPosition(0.5);
        outtakeClaw.scaleRange(0.4, 0.7);
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.setPosition(0.55);
        outtakeWrist.scaleRange(0.25, 0.9);
        outtakeElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        outtakeElbow.setPosition(0.5);
        outtakeElbow.scaleRange(0.2, 0.6);
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        shoulderL.setPosition(0.1);
        shoulderL.scaleRange(0.1, 0.95);
        shoulderR = hardwareMap.get(Servo.class, "shoulderR");
        shoulderR.setPosition(0.9);
        shoulderR.scaleRange(0.05, 0.9);

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setPosition(0.5);
        intakeClaw.scaleRange(0.39, 0.65);
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeWrist.setPosition(0.5);
        intakeWrist.scaleRange(0.18, 0.82);
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeElbow.setPosition(0.6);
        intakeElbow.scaleRange(0.23, 0.76);

        slideMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setPower(1);

        slideMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setTargetPosition(0);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(1);

        pathTimer = new Timer();
        bucketTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Bucket state: ", bucketState);
        telemetry.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState("drop at bucket 1");
        setBucketState("Move to bucket: init");
    }

    public void buildPaths() {
        // Path for scoring preload
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        sample1Snag = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(sample1)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample1.getHeading())
                .build();
        sample2Snag = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(sample2)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample2.getHeading())
                .build();
        sample3Snag = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(sample3)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample3.getHeading())
                .build();

//        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void setBucketState(String bState) {
        bucketState = bState;
        bucketTimer.resetTimer();
    }

    public void setPathState(String pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void extendSlurp() {
    }

    public void retractSlurp() {
    }

    public void bucketDrop(Pose givenPose) {
        switch (bucketState) {
            case "Move to bucket: init": // Move from start to scoring position
                bucketDrop = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(givenPose), new Point(bucketPose)))
                        .setLinearHeadingInterpolation(givenPose.getHeading(), bucketPose.getHeading())
                        .build();
                follower.followPath(bucketDrop, true);
                setBucketState("adjust wrist");
                break;

            case "adjust wrist":
                if (!follower.isBusy()) {
                    setBucketState("open claw");
                }
                break;

            case "open claw":
                if (bucketTimer.getElapsedTime() > bucketClawWait) {
                    setBucketState("retract");
                }
                break;

            case "retract":
                if (bucketTimer.getElapsedTime() > bucketRetractWait) {
                    setBucketState("down");
                }
                break;

            case "down":
                if (bucketTimer.getElapsedTime() > bucketDownWait) {
                    bucketFinished = true;
                }
                break;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "drop at bucket 1":
                bucketDrop(startPose);
                if (bucketFinished) {
                    setPathState("first sample");
                    bucketFinished = false;
                    setBucketState("Move to bucket: init");
                }
                break;

            case "first sample":
                follower.followPath(sample1Snag, true);
                setPathState("extend1");
                break;

            case "extend1":
                if (pathTimer.getElapsedTime() > 2000) {
                    setPathState("retract1");
                }
                break;

            case "retract1":
                if (pathTimer.getElapsedTime() > 1250) {
                    setPathState("grab");
                }
                break;

            case "grab":
                if (pathTimer.getElapsedTime() > clawCloseWait) {
                    setPathState("wrist switch");
                }
                break;

            case "wrist switch":
                if (pathTimer.getElapsedTime() > 200) {
                    setPathState("drop at bucket 2");
                }
                break;

            case "drop at bucket 2":
                bucketDrop(sample1);
                if (bucketFinished) {
                    bucketFinished = false;
                    setPathState("second sample");
                    setBucketState("Move to bucket: init");
                }
                break;

            case "second sample":
                follower.followPath(sample2Snag, true);
                setPathState("extend2");
                break;

            case "extend2":
                if (pathTimer.getElapsedTime() > 2000) {
                    setPathState("retract2");
                }
                break;

            case "retract2":
                if (pathTimer.getElapsedTime() > 1250) {
                    setPathState("grab 2");
                }
                break;

            case "grab 2":
                if (pathTimer.getElapsedTime() > clawCloseWait) {
                    setPathState("wrist switch 2");
                }
                break;

            case "wrist switch 2":
                if (pathTimer.getElapsedTime() > 200) {
                    setPathState("drop at bucket 3");
                }
                break;

            case "drop at bucket 3":
                bucketDrop(sample2);
                if (bucketFinished) {
                    bucketFinished = false;
                    setPathState("third sample");
                    setBucketState("Move to bucket: init");
                }
                break;


            case "third sample":
                follower.followPath(sample3Snag, true);
                setPathState("extend3");
                break;

            case "extend3":
                if (pathTimer.getElapsedTime() > 2000) {
                    setPathState("retract3");
                }
                break;

            case "retract3":
                if (pathTimer.getElapsedTime() > 1250) {
                    setPathState("grab 3");
                }
                break;

            case "grab 3":
                if (pathTimer.getElapsedTime() > clawCloseWaitLast) {
                    setPathState("wrist switch 3");
                }
                break;

            case "wrist switch 3":
                if (pathTimer.getElapsedTime() > 200) {
                    setPathState("drop at bucket 4");
                }
                break;

            case "drop at bucket 4":
                bucketDrop(sample3);
                if (bucketFinished) {
                    bucketFinished = false;
                    setPathState("park");
                }
                break;
        }
    }
}
