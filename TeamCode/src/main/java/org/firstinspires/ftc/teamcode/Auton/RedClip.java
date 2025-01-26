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

@Autonomous(name = "RedClip", group = "Autonomous")
@Config
public class RedClip extends OpMode {

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
    public static double slurpUpperBound = 0.4;
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
    private final Pose clipPose = new Pose(10, 0, Math.toRadians(0));
    private final Pose nPose = new Pose(10, 10, Math.toRadians(0));

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
    private PathChain next, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    Timer opmodeTimer;
    Timer pathTimer;

    @Override
    public void init() {


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

        public void autonomousPathUpdate () {
            switch (pathState) {
                case 0: // Move from start to scoring position
                    follower.followPath(grabPickup1, true);
                    setPathState(1);
                    break;

            case 1: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(next, true);
                    setPathState(2);
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
        }
}
