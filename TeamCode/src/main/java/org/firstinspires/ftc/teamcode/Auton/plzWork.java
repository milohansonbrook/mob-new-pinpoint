package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "buckey or whateva", group = "Autonomous")
@Config
public class plzWork extends OpMode {
    public static double pickUpSpecElbow = 0.68;
    long specStartTime = 0;
    int specStep = 0;
    String pathState = "init";

    //limelight vars :D
    private Limelight3A limelight;
    private LLResult result;
    double[] array;
    double angle;
    double wristPos;
    long time;
    double rangifiedAngle;

    Servo InClaw;//outtake open closed
    Servo InElbow; //outtake claw up/down
    Servo leftHang;
    Servo rightHang;
    DcMotor hangMotorL;
    DcMotor hangMotorR;
    boolean halfSpeed;

    Servo InWrist;//outtake side side
    Servo OutWrist; //Outtake side side
    Servo OutElbow; //Outtake up/down
    Servo OutClaw;//outtake open closed
    Servo rShoulder;
    Servo lShoulder;
    Servo twoBarR;
    Servo twoBarL;
    DcMotor grabMotorL;
    DcMotor grabMotorR;
    boolean outtakeClawOpen = true;


    //Waits
    public static int clawWait = 1000;

    //technical poses


    public static double armMidPosR = 0.63;
    public static double armMidPosL = (1 - armMidPosR);
    public static double barDown = 0.5;
    public static double wristUp = 0.5;
    public static double wristStraight = 0.4;
    public static double backForWall = 0.0;
    public static double OutWristInit = 0.175;
    public static double InElbowInit = 0.6;
    public static double OutElbowInit = 0.4;
    public static int slideWall = 10;
    public static double OutClawInit = 0.98;
    public static double InClawInit = 1;
    public static double clawClose = 1;
    public static double clawOpen = 0;
    public static int slideClipPose1 = 250;
    public static int slideClipPose2 = 650;
    public static double twoBarLGrab = 0.25;
    public static double twoBarRGrab = 0.75;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose bucketDrop = new Pose(3.80568, 20.0666, 5.4525);
    private final Pose grab1Pose = new Pose(17.76188, 9.817, 0.04685);
    private final Pose grab2Pose = new Pose(19.837455449141856, 20.531693201740898, 6.26);
    private final Pose grab3Pose = new Pose(19.002, 12.851, 0.741);
    private final Pose park = new Pose(53.925, -17.822, 4.653);


    //drive poses


    public static double clipPoseX = 27.5;//changed
    public static double clipPoseY = 14.1;
    public static double clipPoseHeading;
    private Follower follower;
    private PathChain bucketPath1, grab1, parkPath, bucketPath2, grab2, bucketPath3, grab3, bucketPath4;
    Timer opmodeTimer;
    Timer pathTimer;
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.start();
        limelight.updatePythonInputs(0,0,0,0,0,0,0,0);

        twoBarL = hardwareMap.get(Servo.class, "intakeBarL");
        twoBarL.scaleRange(0.35, 0.67);
        twoBarL.setPosition(1);
        twoBarR = hardwareMap.get(Servo.class, "intakeBarR");
        twoBarR.scaleRange(0.33, 0.65);
        twoBarR.setPosition(0);

        OutClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        OutClaw.setPosition(1);
        OutClaw.scaleRange(0.4, 0.7);
        OutWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        OutWrist.setPosition(0.55);
        OutWrist.scaleRange(0.25, 0.9);
        OutElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        OutElbow.setPosition(0.75);
        lShoulder = hardwareMap.get(Servo.class, "shoulderL");
        lShoulder.setPosition(0.37);
        lShoulder.scaleRange(0.1, 0.95);
        rShoulder = hardwareMap.get(Servo.class, "shoulderR");
        rShoulder.setPosition(0.63);
        rShoulder.scaleRange(0.05, 0.9);

        InClaw = hardwareMap.get(Servo.class, "intakeClaw");
        InClaw.scaleRange(0.17, 0.65);
        InClaw.setPosition(0.5);
        InWrist = hardwareMap.get(Servo.class, "intakeWrist");
        InWrist.setPosition(0.5);
        InWrist.scaleRange(0.26, 0.76);
        InElbow = hardwareMap.get(Servo.class, "intakeElbow");
        InElbow.setPosition(0.5);

        grabMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        grabMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorL.setTargetPosition(0);
        grabMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorL.setPower(1);

        grabMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        grabMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        grabMotorR.setTargetPosition(0);
        grabMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorR.setPower(1);

        hangMotorL = hardwareMap.get(DcMotor.class, "hangMotorL");
        hangMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotorL.setTargetPosition(0);
        hangMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotorL.setPower(1);

        hangMotorR = hardwareMap.get(DcMotor.class, "hangMotorR");
        hangMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotorR.setTargetPosition(0);
        hangMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotorR.setPower(1);

        leftHang = hardwareMap.get(Servo.class, "turnHangL");
        leftHang.setPosition(0.5);
        rightHang = hardwareMap.get(Servo.class, "turnHangR");
        rightHang.setPosition(0.485);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        halfSpeed = false;

        buildPaths();

    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        result = limelight.getLatestResult();
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    public void start() {
        opmodeTimer.resetTimer();
        //OutWrist.setPosition(0);
        //rShoulder.setPosition(0.9);
        //lShoulder.setPosition(0.1);
//        OutElbow.setPosition(pickUpSpecElbow);
        setPathState("preload");
        //grabMotorR.setTargetPosition(800);
        //grabMotorL.setTargetPosition(800);
    }
    public void buildPaths(){
        bucketPath1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(bucketDrop)))
                .setLinearHeadingInterpolation(startPose.getHeading(), bucketDrop.getHeading())
                .build();
        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketDrop), new Point(grab1Pose)))
                .setLinearHeadingInterpolation(bucketDrop.getHeading(), grab1Pose.getHeading())
                .build();
        bucketPath2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab1Pose), new Point(bucketDrop)))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), bucketDrop.getHeading())
                .build();
        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketDrop), new Point(grab2Pose)))
                .setLinearHeadingInterpolation(bucketDrop.getHeading(), grab2Pose.getHeading())
                .build();
        bucketPath3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab2Pose), new Point(bucketDrop)))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), bucketDrop.getHeading())
                .build();
        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketDrop), new Point(grab3Pose)))
                .setLinearHeadingInterpolation(bucketDrop.getHeading(), grab3Pose.getHeading())
                .build();
        bucketPath4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab3Pose), new Point(bucketDrop)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), bucketDrop.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketDrop), new Point(park)))
                .setLinearHeadingInterpolation(bucketDrop.getHeading(), park.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case "preload":
                twoBarL.setPosition(0.75);
                twoBarR.setPosition(0.25);
                lShoulder.setPosition(1);
                rShoulder.setPosition(0);
                grabMotorL.setTargetPosition(1875);
                grabMotorR.setTargetPosition(1875);
                OutWrist.setPosition(0);
                OutElbow.setPosition(0.9);
                if (!follower.isBusy())
                {
                    follower.followPath(bucketPath1);
                    setPathState("open");
                }
                break;
            case "open":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    OutClaw.setPosition(clawOpen);
                    outtakeClawOpen = true;
                    setPathState("reset arm");
                }
                break;

            case "reset arm":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker reset arm so bot doesn't commit suicide
                    outtakeClawOpen = true;
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InClaw.setPosition(clawOpen);
                    InElbow.setPosition(0.38);
                    InWrist.setPosition(1);
                    OutElbow.setPosition(0.55);
                    OutWrist.setPosition(0.42);
                    lShoulder.setPosition(0.48);
                    rShoulder.setPosition(0.52);
                    setPathState("grab1");
                }
                break;
            case "grab1":
                if (!follower.isBusy())
                {
                    follower.followPath(grab1);
                    setPathState("grab1 arm");
                }
                break;
            case "grab1 arm":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // parker do your thing grab the darn sample
                    twoBarL.setPosition(0.6);
                    twoBarR.setPosition(0.4);
                    InClaw.setPosition(clawOpen);
                    InElbow.setPosition(0.15);
                    InWrist.setPosition(0.5);
                    setPathState("close");
                }
                break;
            case "close":
                if (pathTimer.getElapsedTime() > 1000){
                    InClaw.setPosition(clawClose);
                    setPathState("bring arm1 back");
                }
                break;
            case "bring arm1 back":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // parkerrrr
                    outtakeClawOpen = true;
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InClaw.setPosition(clawClose);
                    InElbow.setPosition(0.85);
                    InWrist.setPosition(1);
                    OutElbow.setPosition(0.55);
                    OutWrist.setPosition(0.42);
                    lShoulder.setPosition(0.48);
                    rShoulder.setPosition(0.52);
                    setPathState("to bucket 2");
                }
                break;
            case "to bucket 2":
                if (!follower.isBusy())
                {
                    follower.followPath(bucketPath2);
                    setPathState("score bucket 2 a");
                }
                break;
            case "score bucket 2 a":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    lShoulder.setPosition(0.52);
                    rShoulder.setPosition(0.48);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0.45);
                    setPathState("score bucket 2 b");
                }
                break;
            case "score bucket 2 b":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InWrist.setPosition(0.5);
                    setPathState("score bucket 2 c");
                }
                break;
            case "score bucket 2 c":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    lShoulder.setPosition(0.45);
                    rShoulder.setPosition(0.55);
                    OutElbow.setPosition(0.6);
                    setPathState("score bucket 2 d");
                }
                break;
            case "score bucket 2 d":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    outtakeClawOpen = false;
                    setPathState("score bucket 2 e");
                }
                break;
            case "score bucket 2 e":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    InClaw.setPosition(clawOpen);
                    setPathState("score bucket 2 f");
                }
                break;
            case "score bucket 2 f":
                if (pathTimer.getElapsedTime() > 1500)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.75);
                    twoBarR.setPosition(0.25);
                    lShoulder.setPosition(1);
                    rShoulder.setPosition(0);
                    grabMotorL.setTargetPosition(1875);
                    grabMotorR.setTargetPosition(1875);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0.9);
                    setPathState("reset arm 2");
                }
                break;
            case "reset arm 2":
                if (pathTimer.getElapsedTime() > 1500)
                {
                    // Parker reset arm so bot doesn't commit suicide
                    outtakeClawOpen = true;
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InClaw.setPosition(clawOpen);
                    InElbow.setPosition(0.38);
                    InWrist.setPosition(1);
                    OutElbow.setPosition(0.55);
                    OutWrist.setPosition(0.42);
                    lShoulder.setPosition(0.48);
                    rShoulder.setPosition(0.52);
                    setPathState("grab2");
                }
                break;
            case "grab2":
                if (!follower.isBusy())
                {
                    follower.followPath(grab2);
                    setPathState("grab2 arm");
                }
                break;
            case "grab2 arm":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // parker do your thing grab the darn sample
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    InClaw.setPosition(clawOpen);
                    InElbow.setPosition(0.38);
                    InWrist.setPosition(0.5);
                    setPathState("close 2");
                }
                break;
            case "close 2":
                if (pathTimer.getElapsedTime() > 1000){
                    InClaw.setPosition(clawClose);
                    setPathState("bring arm2 back");
                }
                break;
            case "bring arm2 back":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // parkerrrr
                    outtakeClawOpen = true;
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InClaw.setPosition(clawClose);
                    InElbow.setPosition(0.85);
                    InWrist.setPosition(1);
                    OutElbow.setPosition(0.55);
                    OutWrist.setPosition(0.42);
                    lShoulder.setPosition(0.48);
                    rShoulder.setPosition(0.52);
                    setPathState("to bucket 3");
                }
                break;
            case "to bucket 3":
                if (!follower.isBusy())
                {
                    follower.followPath(bucketPath3);
                    setPathState("score bucket 3");
                }
                break;
            case "score bucket 3 a":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    lShoulder.setPosition(0.52);
                    rShoulder.setPosition(0.48);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0.45);
                    setPathState("score bucket 3 b");
                }
                break;
            case "score bucket 3 b":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InWrist.setPosition(0.5);
                    setPathState("score bucket 3 c");
                }
                break;
            case "score bucket 3 c":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    lShoulder.setPosition(0.45);
                    rShoulder.setPosition(0.55);
                    OutElbow.setPosition(0.6);
                    setPathState("score bucket 3 d");
                }
                break;
            case "score bucket 3 d":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    outtakeClawOpen = false;
                    setPathState("score bucket 3 e");
                }
                break;
            case "score bucket 3 e":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    InClaw.setPosition(clawOpen);
                    setPathState("score bucket 3 f");
                }
                break;
            case "score bucket 3 f":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.75);
                    twoBarR.setPosition(0.25);
                    lShoulder.setPosition(1);
                    rShoulder.setPosition(0);
                    grabMotorL.setTargetPosition(1850);
                    grabMotorR.setTargetPosition(1850);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0.9);
                    setPathState("reset arm 3");
                }
                break;
            case "reset arm 3":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker reset arm so bot doesn't commit suicide
                    outtakeClawOpen = true;
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InClaw.setPosition(clawOpen);
                    InElbow.setPosition(0.38);
                    InWrist.setPosition(1);
                    OutElbow.setPosition(0.55);
                    OutWrist.setPosition(0.42);
                    lShoulder.setPosition(0.48);
                    rShoulder.setPosition(0.52);
                    setPathState("grab3");
                }
                break;
            case "grab3":
                if (!follower.isBusy())
                {
                    follower.followPath(grab3);
                    setPathState("grab3 arm");
                }
                break;
            case "grab3 arm":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // parker do your thing grab the darn sample
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    InClaw.setPosition(clawOpen);
                    InElbow.setPosition(0.38);
                    InWrist.setPosition(0.5);
                    setPathState("close 3");
                }
                break;
            case "close 3":
                if (pathTimer.getElapsedTime() > 1000){
                    InClaw.setPosition(clawClose);
                    setPathState("bring arm3 back");
                }
                break;
            case "bring arm3 back":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // parkerrrr
                    outtakeClawOpen = true;
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InClaw.setPosition(clawClose);
                    InElbow.setPosition(0.85);
                    InWrist.setPosition(1);
                    OutElbow.setPosition(0.55);
                    OutWrist.setPosition(0.42);
                    lShoulder.setPosition(0.48);
                    rShoulder.setPosition(0.52);
                    setPathState("to bucket 4");
                }
                break;
            case "to bucket 4":
                if (!follower.isBusy())
                {
                    follower.followPath(bucketPath4);
                    setPathState("score bucket 4");
                }
                break;
            case "score bucket 4 a":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    lShoulder.setPosition(0.52);
                    rShoulder.setPosition(0.48);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0.45);
                    setPathState("score bucket 4 b");
                }
                break;
            case "score bucket 4 b":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.85);
                    twoBarR.setPosition(0.15);
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InWrist.setPosition(0.5);
                    setPathState("score bucket 4 c");
                }
                break;
            case "score bucket 4 c":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    lShoulder.setPosition(0.45);
                    rShoulder.setPosition(0.55);
                    OutElbow.setPosition(0.6);
                    setPathState("score bucket 4 d");
                }
                break;
            case "score bucket 4 d":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    outtakeClawOpen = false;
                    setPathState("score bucket 4 e");
                }
                break;
            case "score bucket 4 e":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    InClaw.setPosition(clawOpen);
                    setPathState("score bucket 4 f");
                }
                break;
            case "score bucket 4 f":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker Macro here
                    twoBarL.setPosition(0.75);
                    twoBarR.setPosition(0.25);
                    lShoulder.setPosition(1);
                    rShoulder.setPosition(0);
                    grabMotorL.setTargetPosition(1850);
                    grabMotorR.setTargetPosition(1850);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0.9);
                    setPathState("reset arm 4");
                }
                break;
            case "reset arm 4":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // Parker reset arm so bot doesn't commit suicide
                    outtakeClawOpen = true;
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    InClaw.setPosition(clawOpen);
                    InElbow.setPosition(0.38);
                    InWrist.setPosition(1);
                    OutElbow.setPosition(0.55);
                    OutWrist.setPosition(0.42);
                    lShoulder.setPosition(0.48);
                    rShoulder.setPosition(0.52);
                    setPathState("parkPath");
                }
                break;
            case "parkPath":
                if (!follower.isBusy())
                {
                    follower.followPath(parkPath);
                    setPathState("bar");
                }
                break;
            case "bar":
                if (pathTimer.getElapsedTime() > 1000)
                {
                    // parker your turn again make arm touch bar to get the POINTS YAY
                    setPathState("end");
                }
                break;
            case "end":
                terminateOpModeNow();
                break;


        }
        }



    public void setPathState (String pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}
