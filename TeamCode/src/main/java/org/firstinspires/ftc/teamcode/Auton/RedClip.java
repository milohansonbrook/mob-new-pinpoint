package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "clippy clip", group = "Autonomous")
@Config
public class RedClip extends OpMode {
    public static double pickUpSpecElbow = 0;
    long specStartTime = 0;
    int specStep = 0;
    long specElapsedTime = System.currentTimeMillis() - specStartTime;

    Servo InClaw;//outtake open closed
    Servo InElbow; //outtake claw up/down

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
    Limelight3A limelight;
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
    public static double OutWristInit = 0.0;
    public static double InElbowInit = 0.6;
    public static double OutElbowInit = 0.5;
    public static int slideWall = 10;
    public static double OutClawInit = 0.5;
    public static double InClawInit = 0.5;
    public static double clawClose = 1;
    public static double clawOpen = 0;
    public static int slideClipPose1 = 250;
    public static int slideClipPose2 = 650;


    //drive poses
    public static double clipPoseX = 25;
    public static double clipPoseY = 11.1;
    public static double clipPoseHeading;

    String pathState = "init";
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose clip1 = new Pose(clipPoseX, clipPoseY, Math.toRadians(0));
    private final Pose clip2 = new Pose(23.8, 13.1, Math.toRadians(0));
    private final Pose clip3 = new Pose(23.8, 15.1, Math.toRadians(0));
    private final Pose clip4 = new Pose(23.8, 17.1, Math.toRadians(0));
    private final Pose clip5 = new Pose(23.8, 19.1, Math.toRadians(0));
    private final Pose prePush1 = new Pose(47.8, -32, Math.toRadians(0)); //front
    private final Pose helper = new Pose(21.5, -14, Math.toRadians(0)); //like near post
    private final Pose set1 = new Pose(46, -22, Math.toRadians(0)); //left front of samples
    private final Pose observe1 = new Pose(7.7, -30, Math.toRadians(0)); //ob zone
    private final Pose set2 = new Pose(47, -30, Math.toRadians(0));
    private final Pose prePush2 = new Pose(46.5, -33, Math.toRadians(0));
    private final Pose observe2 = new Pose(10.5, -33, Math.toRadians(0));
    private final Pose set3 = new Pose(47.3, -33, Math.toRadians(0));
    private final Pose prePush3 = new Pose(46.7, -36, Math.toRadians(0));
    private final Pose observe3 = new Pose(11.3, -36, Math.toRadians(0));
    private final Pose grabPos = new Pose(1.8, -37.5, Math.toRadians(0));



    private Follower follower;
    private PathChain bar1, pushPoint1, pushPoint2, pushPoint3, pushPoint4, pushPoint5, pushPoint6, pushPoint7, pushPoint8, pushPoint9, pushPoint10, grab1, grab2, grab3, bar2, bar3, bar4;
    Timer opmodeTimer;
    Timer pathTimer;

    @Override
    public void init() {




        twoBarL = hardwareMap.get(Servo.class, "intakeBarL");
        twoBarL.scaleRange(0.425, 0.72);
        twoBarL.setPosition(1);

        twoBarR = hardwareMap.get(Servo.class, "intakeBarR");
        twoBarR.scaleRange(0.29, 0.585);
        twoBarR.setPosition(0);

        InWrist = hardwareMap.get(Servo.class, "intakeWrist");
        InWrist.scaleRange(0.18, 0.82);
        InWrist.setPosition(wristUp);

        OutWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        OutWrist.scaleRange(0.25, 0.9);
        //OutWrist.setPosition(OutWristInit);

        InElbow = hardwareMap.get(Servo.class, "intakeElbow");

        InElbow.setPosition(InElbowInit);

        OutElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        OutElbow.scaleRange(0.2, 0.6);
        //OutElbow.setPosition(OutElbowInit);

        rShoulder = hardwareMap.get(Servo.class, "shoulderR");
        rShoulder.scaleRange(0.05, 0.9);
        rShoulder.setPosition(armMidPosR);

        lShoulder = hardwareMap.get(Servo.class, "shoulderL");
        lShoulder.scaleRange(0.1, 0.95);
        lShoulder.setPosition(armMidPosL);

        grabMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        grabMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorL.setTargetPosition(0);
        grabMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorL.setPower(0.5);

        grabMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        grabMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        grabMotorR.setTargetPosition(0);
        grabMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorR.setPower(0.5);

        InClaw = hardwareMap.get(Servo.class, "intakeClaw");
        InClaw.scaleRange(0.39, 0.65);
        InClaw.setPosition(InClawInit);

        OutClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        OutClaw.scaleRange(0.4, 0.7);
        OutClaw.setPosition(OutClawInit);
        OutWrist.setPosition(0);
        OutElbow.setPosition(pickUpSpecElbow);



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
        setPathState("set slurp");
    }

    public void buildPaths() {
        // Path for scoring preload
        bar1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(clip1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), clip1.getHeading())
                .build();
        pushPoint1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clip1), new Point(helper)))
                .setLinearHeadingInterpolation(clip1.getHeading(), helper.getHeading())
                .build();
        pushPoint2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(helper), new Point(set1)))
                .setLinearHeadingInterpolation(helper.getHeading(), set1.getHeading())
                .build();
        pushPoint3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(set1), new Point(prePush1)))
                .setLinearHeadingInterpolation(set1.getHeading(), prePush1.getHeading())
                .build();
        pushPoint4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prePush1), new Point(observe1)))
                .setLinearHeadingInterpolation(prePush1.getHeading(), observe1.getHeading())
                .build();
        pushPoint5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observe1), new Point(set2)))
                .setLinearHeadingInterpolation(observe1.getHeading(), set2.getHeading())
                .build();
        pushPoint6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(set2), new Point(prePush2)))
                .setLinearHeadingInterpolation(set2.getHeading(), prePush2.getHeading())
                .build();
        pushPoint7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prePush2), new Point(observe2)))
                .setLinearHeadingInterpolation(prePush2.getHeading(), observe2.getHeading())
                .build();
        pushPoint8 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observe2), new Point(set3)))
                .setLinearHeadingInterpolation(observe2.getHeading(), set3.getHeading())
                .build();
        pushPoint9 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(set3), new Point(prePush3)))
                .setLinearHeadingInterpolation(set3.getHeading(), prePush3.getHeading())
                .build();
        pushPoint10 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prePush3), new Point(observe3)))
                .setLinearHeadingInterpolation(prePush3.getHeading(), observe3.getHeading())
                .build();
        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observe3), new Point(grabPos)))
                .setLinearHeadingInterpolation(observe3.getHeading(), grabPos.getHeading())
                .build();
        bar2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPos), new Point(clip2)))
                .setLinearHeadingInterpolation(grabPos.getHeading(), clip2.getHeading())
                .build();
        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clip2), new Point(grabPos)))
                .setLinearHeadingInterpolation(clip2.getHeading(), grabPos.getHeading())
                .build();
        bar3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPos), new Point(clip3)))
                .setLinearHeadingInterpolation(grabPos.getHeading(), clip3.getHeading())
                .build();
        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clip3), new Point(grabPos)))
                .setLinearHeadingInterpolation(clip3.getHeading(), grabPos.getHeading())
                .build();
        bar4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPos), new Point(clip4)))
                .setLinearHeadingInterpolation(grabPos.getHeading(), clip4.getHeading())
                .build();

    }
    public void setBarPose(double pose){
        lShoulder.setPosition(pose);
        rShoulder.setPosition(1 - pose);
    }
    public void setVertSlide(int pose){
        grabMotorL.setTargetPosition(pose);
        grabMotorR.setTargetPosition(pose);
    }

        public void autonomousPathUpdate() {
            switch (pathState) {
                case "set wrist":
                    if (!follower.isBusy()){
                    follower.followPath(bar1, true);

                    setPathState("move to bar");
                    }
                    break;

                case "move to bar":

                        OutWrist.setPosition(0);
                        OutElbow.setPosition(pickUpSpecElbow);
                        if (specElapsedTime >= 2000) {
                            specStep++;
                            specStartTime = System.currentTimeMillis();
                        setPathState("clip");
                    }
                    break;

                case "clip":
                    outtakeClawOpen = false;
                    OutClaw.setPosition(1);
                    if (specElapsedTime >= 3000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("next clip step");
                    break;
                case "next clip step" :
                    grabMotorR.setTargetPosition(600);
                    grabMotorL.setTargetPosition(600);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("another clip step");
                    break;
                case "another clip step":
                    OutElbow.setPosition(1);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("yet another");
                    break;
                case "yet another":
                    rShoulder.setPosition(0.5);
                    lShoulder.setPosition(0.5);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }

                    setPathState("two to go");
                    break;
                case "two to go":
                    grabMotorR.setTargetPosition(1640);
                    grabMotorL.setTargetPosition(1640);
                    if (specElapsedTime >= 700)
                    {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("last clip");
                    break;
                case "last clip":
                    OutClaw.setPosition(clawOpen);
                    outtakeClawOpen = true;
                    setPathState("open claw");
                    break;
                case "open claw":
                    if (pathTimer.getElapsedTime() > clawWait){

                        OutClaw.setPosition(clawOpen);

                        setPathState("move set1");
                    }
                    break;

                case "move set1":
                    setVertSlide(slideWall);

                    follower.followPath(pushPoint1, true);
                    rShoulder.setPosition(0.9);
                    lShoulder.setPosition(0.1);
                    twoBarL.setPosition(1);
                    twoBarR.setPosition(0);
                    follower.followPath(pushPoint2, true);
                    setPathState("push back1");
                    break;

                case "push back1":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint3, true);
                        follower.followPath(pushPoint4, true);
                        setPathState("move set2");
                    }
                    break;

                case "move set2":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint5, true);
                        follower.followPath(pushPoint6, true);
                        setPathState("Back + set3");
                    }
                    break;

                case "Back + set3":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint7, true);
                        follower.followPath(pushPoint8, true);
                        setPathState("Back4");
                    }
                    break;

                case "Back4":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint9, true);
                        follower.followPath(pushPoint10, true);
                        setPathState("Grab1");
                    }
                    break;
                case "Grab1":
                    if(!follower.isBusy()){
                        OutClaw.setPosition(clawOpen);
                        outtakeClawOpen = true;
                        follower.followPath(grab1, true);
                        setPathState("arm grab");
                    }
                    break;
                case "arm grab":
                    //arm movement to grab off wall

                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    rShoulder.setPosition(0.9);
                    lShoulder.setPosition(0.1);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0);
                    OutClaw.setPosition(clawClose);

                    setPathState("move");
                    break;
                case "move":
                    if (!follower.isBusy()) {
                        follower.followPath(bar2, true);
                        setPathState("move to bar2");
                    }
                case "move to bar2":

                    /*OutWrist.setPosition(0);
                    OutElbow.setPosition(pickUpSpecElbow);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();

                    }*/
                    setPathState("clip2");
                    break;

                case "clip2":
                    outtakeClawOpen = false;
                    OutClaw.setPosition(1);
                    if (specElapsedTime >= 3000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("next clip step2");
                    break;
                case "next clip step2" :
                    grabMotorR.setTargetPosition(600);
                    grabMotorL.setTargetPosition(600);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("another clip step2");
                    break;
                case "another clip step2":
                    OutElbow.setPosition(1);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("yet another2");
                    break;
                case "yet another2":
                    rShoulder.setPosition(0.5);
                    lShoulder.setPosition(0.5);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("two to go2");
                    break;
                case "two to go2":
                    grabMotorR.setTargetPosition(1640);
                    grabMotorL.setTargetPosition(1640);
                    if (specElapsedTime >= 700)
                    {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("last clip2");
                    break;
                case "last clip2":
                    OutClaw.setPosition(clawOpen);
                    outtakeClawOpen = true;
                    setPathState("open claw2");
                    break;

                case "open claw2":
                    if (pathTimer.getElapsedTime() > clawWait){

                        OutClaw.setPosition(clawOpen);

                        setPathState("Grab2");
                    }
                    break;

                case "Grab2":
                    if (!follower.isBusy())
                    {
                        OutClaw.setPosition(clawOpen);
                        outtakeClawOpen = true;
                        follower.followPath(grab2, true);
                        setPathState("arm grab2");
                    }
                    break;
                case "arm grab2":
                    //arm movement to grab off wall
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    rShoulder.setPosition(0.9);
                    lShoulder.setPosition(0.1);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0);
                    OutClaw.setPosition(clawClose);

                    setPathState("move2");
                    break;
                case "move2":
                    if (!follower.isBusy()) {
                        follower.followPath(bar3, true);
                        setPathState("move to bar3");
                    }
                case "move to bar3":

                   /* OutWrist.setPosition(0);
                    OutElbow.setPosition(pickUpSpecElbow);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();

                    }*/
                    setPathState("clip3");
                    break;

                case "clip3":
                    outtakeClawOpen = false;
                    OutClaw.setPosition(1);
                    if (specElapsedTime >= 3000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("next clip step3");
                    break;
                case "next clip step3" :
                    grabMotorR.setTargetPosition(600);
                    grabMotorL.setTargetPosition(600);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("another clip step3");
                    break;
                case "another clip step3":
                    OutElbow.setPosition(1);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("yet another3");
                    break;
                case "yet another3":
                    rShoulder.setPosition(0.5);
                    lShoulder.setPosition(0.5);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("two to go3");
                    break;
                case "two to go3":
                    grabMotorR.setTargetPosition(1640);
                    grabMotorL.setTargetPosition(1640);
                    if (specElapsedTime >= 700)
                    {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("last clip3");
                    break;
                case "last clip3":
                    OutClaw.setPosition(clawOpen);
                    outtakeClawOpen = true;
                    setPathState("open claw3");
                    break;

                case "open claw3":
                    if (pathTimer.getElapsedTime() > clawWait){

                        OutClaw.setPosition(clawOpen);

                        setPathState("Grab3");
                    }
                    break;
                case "Grab3":
                    if (!follower.isBusy())
                    {
                        OutClaw.setPosition(clawOpen);
                        outtakeClawOpen = true;
                        follower.followPath(grab3, true);
                        setPathState("arm grab3");
                    }
                    break;
                case "arm grab3":
                    //arm movement to grab off wall
                    grabMotorL.setTargetPosition(0);
                    grabMotorR.setTargetPosition(0);
                    rShoulder.setPosition(0.9);
                    lShoulder.setPosition(0.1);
                    OutWrist.setPosition(0);
                    OutElbow.setPosition(0);
                    OutClaw.setPosition(clawClose);
                    setPathState("move3");
                    break;
                case "move3":
                    if (!follower.isBusy()) {
                        follower.followPath(bar4, true);
                        setPathState("move to bar4");
                    }
                case "move to bar4":

                    /*OutWrist.setPosition(0);
                    OutElbow.setPosition(pickUpSpecElbow);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();

                    }*/
                    setPathState("clip4");
                    break;

                case "clip4":
                    outtakeClawOpen = false;
                    OutClaw.setPosition(1);
                    if (specElapsedTime >= 3000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("next clip step4");
                    break;
                case "next clip step4" :
                    grabMotorR.setTargetPosition(600);
                    grabMotorL.setTargetPosition(600);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("another clip step4");
                    break;
                case "another clip step4":
                    OutElbow.setPosition(1);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("yet another4");
                    break;
                case "yet another4":
                    rShoulder.setPosition(0.5);
                    lShoulder.setPosition(0.5);
                    if (specElapsedTime >= 2000) {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("two to go4");
                    break;
                case "two to go4":
                    grabMotorR.setTargetPosition(1640);
                    grabMotorL.setTargetPosition(1640);
                    if (specElapsedTime >= 700)
                    {
                        specStep++;
                        specStartTime = System.currentTimeMillis();
                    }
                    setPathState("last clip4");
                    break;
                case "last clip4":
                    OutClaw.setPosition(clawOpen);
                    outtakeClawOpen = true;
                    setPathState("open claw4");
                    break;

                case "open claw4":
                    if (pathTimer.getElapsedTime() > clawWait){

                        OutClaw.setPosition(clawOpen);

                        setPathState("end");
                    }
                    break;

            }
        }
        public void setPathState (String pState){
            pathState = pState;
            pathTimer.resetTimer();
        }
}