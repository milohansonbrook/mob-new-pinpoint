package org.firstinspires.ftc.teamcode.Auton;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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
@Autonomous (name="Auton Bucket", group = "Autonomous")
public class JuliannaBucket extends OpMode {
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
    //private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose bucketPose = new Pose(5.6, 21.36, Math.toRadians(-45));
    private final Pose nPose = new Pose(6.05, 12.67, Math.toRadians(0));

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    //private int pathState;
    private PathChain scorePreload, park;
    private PathChain slurp1, slurp2, slurp3, score1, score2, score3;
    private final Pose startPose = new Pose(134, 45, Math.toRadians(180));
    private final Pose slurp1Pose = new Pose(114, 21, Math.toRadians(180));
    private final Pose score1Pose = new Pose(127, 19, Math.toRadians(150));
    private final Pose slurp2Pose = new Pose(114, 13, Math.toRadians(180));
    private final Pose score2Pose = new Pose(127, 19, Math.toRadians(150));
    private final Pose slurp3Pose = new Pose(114, 7, Math.toRadians(180));
    private final Pose score3Pose = new Pose(127,19, Math.toRadians(150));
    //private final Pose grabPose = new Pose(133, 120, Math.toRadians(180));
    //private final Pose clipPose2 = new Pose(100, 76, Math.toRadians(-180));
    //private final Pose clipPose3 = new Pose(100, 74, Math.toRadians(-180));
   //private final Pose clipPose4 = new Pose(100, 72, Math.toRadians(-180));
    private final Pose end = new Pose(83, 43, Math.toRadians(180));
    private Servo Lshoulder, Rshoulder, turnGrabber, openGrabber;
    private DcMotor Lslide, Rslide, out;

    public void buildPaths()
    {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading())
                .build();
        slurp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(slurp1Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), slurp1Pose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slurp1Pose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(slurp1Pose.getHeading(), score1Pose.getHeading())
                .build();
        slurp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(slurp2Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), slurp2Pose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slurp2Pose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(slurp2Pose.getHeading(), score2Pose.getHeading())
                .build();
        slurp3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(slurp3Pose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), slurp3Pose.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slurp3Pose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(slurp3Pose.getHeading(), score3Pose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3Pose), new Point(88, 24),new Point(end)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), end.getHeading())
                .build();

    }
    public void setPathState(String pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "zero":
                follower.followPath(scorePreload);
                setPathState("1st score");
                break;
            case "one":
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(slurp1,true);
                    setPathState("1st slurp");
                }
                break;
            case "two":
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score1,true);
                    setPathState("2nd score");
                }
                break;
            case "three":
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(slurp2,true);
                    setPathState("2nd slurp");
                }
                break;
            case "four":
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    setPathState("3rd score");
                }
                break;
            case "five":
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(slurp3,true);
                    setPathState("3rd slurp");
                }
                break;
            case "six":
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score3, true);
                    setPathState("4th score");
                }
                break;
            case "seven":
                if(!follower.isBusy())
                {
                    follower.followPath(park, true);
                    setPathState("park");
                }
            case "eight":
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState("Not used");
                }
                break;
        }
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

    /** This method is called once at the init of the OpMode. **/
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
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState("zero");
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
