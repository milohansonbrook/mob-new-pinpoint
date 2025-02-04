package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.AutonPoses.RedClipPoses; //to use
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "AlexBucket", group = "Autonomous")
@Config
public class RedBucket extends OpMode {

    Servo turnSlurp;
    Servo claw;
    Servo turnClaw;
    Servo rShoulder;
    Servo lShoulder;
    Servo twoBarR;
    Servo twoBarL;
    CRServo slurp;
    DcMotor grabMotorL;
    DcMotor grabMotorR;

    public static double slurpLowerBound = 0.21;
    public static double slurpUpperBound = 0.4;
    public static int clawHeight = 860;
    public static double armMidPosL = 0.81;
    public static double armMidPosR = (1 - armMidPosL);
    public static double wristMidPos = 0.5;

    boolean intakeDown;
    boolean aLast;
    boolean clawState;
    boolean sampleSequenceActive;
    boolean sampleSequenceComplete;
    boolean specimenSequenceActive;
    boolean specimenSequenceComplete;
    boolean yLast;
    boolean jiggle;
    boolean stickB1Last;
    boolean halfSpeed;
    double drivePower;
    long sampleSequenceStartTime = 0;
    long specimenSequenceStartTime = 0;
    int sampleSequenceStep = 0;
    int specimenSequenceStep = 0;


    String pathState = "init";
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose bucketPose = new Pose(5.6, 21.36, Math.toRadians(-45));
    private final Pose nPose = new Pose(6.05, 12.67, Math.toRadians(0));

    // Starting position
//    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315)); // Scoring position
//
//    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // First sample pickup
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Second sample pickup
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Third sample pickup
//
//    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
//    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90)); // Control point for curved path

    private Follower follower;
    private Path scorePreload, park;
    private PathChain bucketDrop, next, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    Timer opmodeTimer;
    Timer pathTimer;

    @Override
    public void init() {
        clawState = true;

        turnSlurp = hardwareMap.get(Servo.class, "turnSlurp");
        turnSlurp.scaleRange(slurpLowerBound, slurpUpperBound);
        turnSlurp.setPosition(1);

        twoBarL = hardwareMap.get(Servo.class, "twoBarL");
        twoBarL.scaleRange(0.425, 0.72);
        twoBarL.setPosition(1);

        twoBarR = hardwareMap.get(Servo.class, "twoBarR");
        twoBarR.scaleRange(0.29, 0.585);
        twoBarR.setPosition(0);

        turnClaw = hardwareMap.get(Servo.class, "turnClaw");
        turnClaw.scaleRange(0, 1);
        turnClaw.setPosition(wristMidPos);

        rShoulder = hardwareMap.get(Servo.class, "rShoulder");
        rShoulder.scaleRange(0, 1);
        rShoulder.setPosition(armMidPosR);

        lShoulder = hardwareMap.get(Servo.class, "lShoulder");
        lShoulder.scaleRange(0, 1);
        lShoulder.setPosition(armMidPosL);

        grabMotorL = hardwareMap.get(DcMotor.class, "grabMotorL");
        grabMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorL.setTargetPosition(0);
        grabMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorL.setPower(0.5);

        grabMotorR = hardwareMap.get(DcMotor.class, "grabMotorR");
        grabMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        grabMotorR.setTargetPosition(0);
        grabMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorR.setPower(0.5);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0.525, 0.64);
        claw.setPosition(0);

        slurp = hardwareMap.get(CRServo.class, "slurp");

        pathTimer = new Timer();
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
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState("Move to bucket: init");
    }

    public void buildPaths() {
        // Path for scoring preload
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        bucketDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), bucketPose.getHeading())
                .build();

//        next = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(clipPose), new Point(nPose)))
//                .setLinearHeadingInterpolation(clipPose.getHeading(), nPose.getHeading())
//                .build();

//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "Move to bucket: init": // Move from start to scoring position
                    follower.followPath(bucketDrop, true);
                    setPathState("Grab sample");
                break;

            case "234234":
                if (!follower.isBusy()) {
                    setPathState("");
                    grabMotorL.setTargetPosition(375);
                    grabMotorR.setTargetPosition(375);
                }
                break;

            case "32423": // Wait until the robot is near the first sample pickup position
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(next, true);
                    setPathState("83");
                }
                break;

//            case 2: // Wait until the robot is near the first sample pickup position
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup1, true);
//                    setPathState(3);
//                }
//                break;
//
//            case 3: // Wait until the robot returns to the scoring position
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
//                }
//                break;
//
//            case 4: // Wait until the robot is near the second sample pickup position
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup2, true);
//                    setPathState(5);
//                }
//                break;
//
//            case 5: // Wait until the robot returns to the scoring position
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup3, true);
//                    setPathState(6);
//                }
//                break;
//
//            case 6: // Wait until the robot is near the third sample pickup position
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup3, true);
//                    setPathState(7);
//                }
//                break;
//
//            case 7: // Wait until the robot returns to the scoring position
//                if (!follower.isBusy()) {
//                    follower.followPath(park, true);
//                    setPathState(8);
//                }
//                break;
//
//            case 8: // Wait until the robot is near the parking position
//                if (!follower.isBusy()) {
//                    setPathState(-1); // End the autonomous routine
//                }
//                break;
        }
    }
    public void setPathState (String pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}

