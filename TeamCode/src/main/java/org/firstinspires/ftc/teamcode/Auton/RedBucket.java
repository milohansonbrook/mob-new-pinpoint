package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonPoses.RedClipPoses; //to use
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    //Hardware
    Servo turnSlurp;
    Servo claw;
    Servo wrist;
    Servo rShoulder;
    Servo lShoulder;
    Servo twoBarR;
    Servo twoBarL;
    CRServo slurp;
    DcMotor grabMotorL;
    DcMotor grabMotorR;

    //Time variables:
    public static int bucketWristWait = 900;
    public static int bucketClawWait = 400;
    public static int bucketRetractWait = 400;
    public static int bucketDownWait = 400;
    public static int clawCloseWait = 700;

    public static double slurpLowerBound = 0.21;
    public static double slurpUpperBound = 0.65;

    //Pose variables
    public static double clawClose = 1;
    public static double clawOpen = 0;
    public static double lShoulderUp = 0;
    public static double rShoulderUp = 1;
    public static double lShoulderSnag = 0.8;
    public static double rShoulderSnag = 1-lShoulderSnag;
    public static double wristUp = 1;
    public static double wristStraight = 0.5;
    public static double extendBarL = 0;
    public static double extendBarR = 1;
    public static double slurpDefault = 0.15;
    public static double slurpDown = 0.025;
    public static double slurpUp = 0.5;
    public static double retractBarL = 1;
    public static double retractBarR = 0;

    public static double sample3x = 13;
    public static double sample3y = 22;
    public static double sample3heading = 22;


    public static int bucketSlidePos = 850;

    public boolean bucketFinished = false;

    public String bucketState = "init";
    public String pathState = "init";
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose sample1 = new Pose(8.48, 10.76, Math.toRadians(0));
    private final Pose sample2 = new Pose(8.48, 19, Math.toRadians(0));
    private final Pose sample3 = new Pose(sample3x, sample3y, Math.toRadians(sample3heading));
    private final Pose bucketPose = new Pose(5.3, 18, Math.toRadians(315));

    private Follower follower;
    private PathChain bucketDrop, sample1Snag, sample2Snag, sample3Snag;
    Timer opmodeTimer;
    Timer pathTimer;
    Timer bucketTimer;

    @Override
    public void init() {

        turnSlurp = hardwareMap.get(Servo.class, "turnSlurp");
        turnSlurp.scaleRange(slurpLowerBound, slurpUpperBound);
        turnSlurp.setPosition(1);

        twoBarL = hardwareMap.get(Servo.class, "twoBarL");
        twoBarL.scaleRange(0.425, 0.72);
        twoBarL.setPosition(1);

        twoBarR = hardwareMap.get(Servo.class, "twoBarR");
        twoBarR.scaleRange(0.29, 0.585);
        twoBarR.setPosition(0);

        wrist = hardwareMap.get(Servo.class, "turnClaw");
        wrist.scaleRange(0, 1);
        wrist.setPosition(wristUp);

        rShoulder = hardwareMap.get(Servo.class, "rShoulder");
        rShoulder.scaleRange(0, 1);
        rShoulder.setPosition(0.55);

        lShoulder = hardwareMap.get(Servo.class, "lShoulder");
        lShoulder.scaleRange(0, 1);
        lShoulder.setPosition(0.45);

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
        claw.setPosition(clawClose);

        slurp = hardwareMap.get(CRServo.class, "slurp");

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

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
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
    public void setBucketState(String bState){
        bucketState = bState;
        bucketTimer.resetTimer();
    }

    public void setPathState (String pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void extendSlurp(){
        twoBarL.setPosition(extendBarL);
        twoBarR.setPosition(extendBarR);
        turnSlurp.setPosition(slurpDown);
        slurp.setPower(-1);
    }

    public void retractSlurp(){
        twoBarL.setPosition(retractBarL);
        twoBarR.setPosition(retractBarR);
        turnSlurp.setPosition(slurpUp);
    }

    public void bucketDrop(Pose givenPose){
        switch (bucketState){
            case "Move to bucket: init": // Move from start to scoring position
                bucketDrop = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(givenPose), new Point(bucketPose)))
                        .setLinearHeadingInterpolation(givenPose.getHeading(), bucketPose.getHeading())
                        .build();
                follower.followPath(bucketDrop, true);
                grabMotorL.setTargetPosition(bucketSlidePos);
                grabMotorR.setTargetPosition(bucketSlidePos);
                lShoulder.setPosition(lShoulderUp);
                rShoulder.setPosition(rShoulderUp);
                setBucketState("adjust wrist");
                break;

            case "adjust wrist":
                if (!follower.isBusy()) {
                    wrist.setPosition(wristStraight);
                    setBucketState("open claw");
                }
                break;

            case "open claw":
                if (bucketTimer.getElapsedTime() > bucketClawWait) {
                    claw.setPosition(clawOpen);
                    setBucketState("retract");
                }
                break;

            case "retract":
                if (bucketTimer.getElapsedTime() > bucketRetractWait){
                    wrist.setPosition(wristUp);
                    setBucketState("down");
                }
                break;

            case "down":
                if (bucketTimer.getElapsedTime() > bucketDownWait){
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    lShoulder.setPosition(0.5);
                    rShoulder.setPosition(0.5);
                    bucketFinished = true;
                }
                break;
        }
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case "drop at bucket 1":
                turnSlurp.setPosition(slurpDefault);
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
                    rShoulder.setPosition(rShoulderSnag);
                    lShoulder.setPosition(lShoulderSnag);
                    wrist.setPosition(wristStraight);
                    extendSlurp();
                    setPathState("retract1");
                }
                break;

            case "retract1":
                if (pathTimer.getElapsedTime() > 1250) {
                    retractSlurp();
                    setPathState("grab");
                }
                break;

            case "grab":
                if (pathTimer.getElapsedTime() > clawCloseWait) {
                    slurp.setPower(0);
                    claw.setPosition(clawClose);
                    turnSlurp.setPosition(0.5);
                    setPathState("wrist switch");
                }
                break;

            case "wrist switch":
                if (pathTimer.getElapsedTime() > 200) {
                    wrist.setPosition(wristUp);
                    setPathState("drop at bucket 2");
                }
                break;

            case "drop at bucket 2":
                bucketDrop(sample1);
                if (bucketFinished){
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
                    rShoulder.setPosition(rShoulderSnag);
                    lShoulder.setPosition(lShoulderSnag);
                    wrist.setPosition(wristStraight);
                    extendSlurp();
                    setPathState("retract2");
                }
                break;

            case "retract2":
                if (pathTimer.getElapsedTime() > 1250) {
                    retractSlurp();
                    setPathState("grab 2");
                }
                break;

            case "grab 2":
                if (pathTimer.getElapsedTime() > clawCloseWait) {
                    slurp.setPower(0);
                    claw.setPosition(clawClose);
                    turnSlurp.setPosition(0.5);
                    setPathState("wrist switch 2");
                }
                break;

            case "wrist switch 2":
                if (pathTimer.getElapsedTime() > 200) {
                    wrist.setPosition(wristUp);
                    setPathState("drop at bucket 3");
                }
                break;

            case "drop at bucket 3":
                bucketDrop(sample2);
                if (bucketFinished){
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
                    rShoulder.setPosition(rShoulderSnag);
                    lShoulder.setPosition(lShoulderSnag);
                    wrist.setPosition(wristStraight);
                    extendSlurp();
                    setPathState("retract3");
                }
                break;

            case "retract3":
                if (pathTimer.getElapsedTime() > 1250) {
                    retractSlurp();
                    setPathState("grab 3");
                }
                break;

            case "grab 3":
                if (pathTimer.getElapsedTime() > clawCloseWait) {
                    slurp.setPower(0);
                    claw.setPosition(clawClose);
                    turnSlurp.setPosition(0.5);
                    setPathState("wrist switch 3");
                }
                break;

            case "wrist switch 3":
                if (pathTimer.getElapsedTime() > 200) {
                    wrist.setPosition(wristUp);
                    setPathState("drop at bucket 4");
                }
                break;

            case "drop at bucket 4":
                bucketDrop(sample3);
                if (bucketFinished){
                    bucketFinished = false;
                    setPathState("park");
                }
                break;
        }
    }

}

