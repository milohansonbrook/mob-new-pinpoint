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

@Autonomous(name = "RedBucket", group = "Autonomous")
@Config
public class RedBucket extends OpMode {

    Servo intakeRight;
    Servo claw;
    Servo outtakeWrist;
    Servo outtakeArmRight;
    Servo outtakeArmLeft;
    Servo twoBarR;
    Servo twoBarL;
    CRServo slurp;
    DcMotor grabMotorL;
    DcMotor grabMotorR;
    public static double slurpLowerBound = 0.195;
    public static double slurpUpperBound = 0.6;
    public static int clawHeight = 860;
    public static double armMidPosL = 0.81;
    public static double armMidPosR = (1 - armMidPosL);
    public static double wristMidPos = 0.5;
    boolean intakeDown;
    boolean aLast;
    boolean clawState;
    boolean armSequenceActive;
    boolean armSequenceComplete;
    boolean sampleSeqActive;
    boolean sampleSeqComplete;
    boolean yLast;
    boolean jiggle;
    boolean b2Last;
    boolean halfSpeed;
    double drivePower;
    long sequenceStartTime = 0;
    long sampleSeqStartTime = 0;
    int armSequenceStep = 0;
    int sampleSeqStep = 0;

    int pathState = 0;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose clipPose = new Pose(24, 26, Math.toRadians(-45));
    private final Pose nPose = new Pose(13.7, 37.8, Math.toRadians(-45));

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
    private PathChain grabPickup1, next, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    Timer opmodeTimer;
    Timer pathTimer;

    @Override
    public void init() {
        clawState = true;

        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeRight.scaleRange(slurpLowerBound, slurpUpperBound);
        intakeRight.setPosition(1);

        twoBarL = hardwareMap.get(Servo.class, "twoBarL");
        twoBarL.scaleRange(0.425, 0.69);
        twoBarL.setPosition(1);

        twoBarR = hardwareMap.get(Servo.class, "twoBarR");
        twoBarR.scaleRange(0.31, 0.575);
        twoBarR.setPosition(0);

        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.scaleRange(0, 1);
        outtakeWrist.setPosition(wristMidPos);

        outtakeArmRight = hardwareMap.get(Servo.class, "outtakeArmRight");
        outtakeArmRight.scaleRange(0, 1);
        outtakeArmRight.setPosition(armMidPosR);

        outtakeArmLeft = hardwareMap.get(Servo.class, "outtakeArmLeft");
        outtakeArmLeft.scaleRange(0, 1);
        outtakeArmLeft.setPosition(armMidPosL);

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

        halfSpeed = false;

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
        setPathState(0);
    }

    public void buildPaths() {
        // Path for scoring preload
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(clipPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), clipPose.getHeading())
                .build();

        next = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clipPose), new Point(nPose)))
                .setLinearHeadingInterpolation(clipPose.getHeading(), nPose.getHeading())
                .build();

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
            case 0: // Move from start to scoring position
                    intakeRight.setPosition(0.45);
                    follower.followPath(grabPickup1, true);
                    outtakeArmRight.setPosition(0.5);
                    outtakeArmLeft.setPosition(0.5);
//                    outtakeWrist.setPosition(0.35);
                    setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                    grabMotorL.setTargetPosition(375);
                    grabMotorR.setTargetPosition(375);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(next, true);
                    setPathState(3);
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
    public void setPathState (int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}

