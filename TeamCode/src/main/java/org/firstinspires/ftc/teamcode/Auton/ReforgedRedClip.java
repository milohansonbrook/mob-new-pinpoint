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


@Autonomous(name = "clippy clip Reforged", group = "Autonomous")
@Config
public class ReforgedRedClip extends OpMode {
    public static double pickUpSpecElbow = 0.68;
    long specStartTime = 0;
    int specStep = 0;

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
    public static double InClawInit = 0.5;
    public static double clawClose = 1;
    public static double clawOpen = 0;
    public static int slideClipPose1 = 250;
    public static int slideClipPose2 = 650;
    public static double twoBarLGrab = 0.25;
    public static double twoBarRGrab = 0.75;


    //drive poses
    public static double clipPoseX = 27.5;//changed
    public static double clipPoseY = 11.1;
    public static double clipPoseHeading;

    String pathState = "init";
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose clip1 = new Pose(clipPoseX, clipPoseY, Math.toRadians(0));
    private final Pose clip2 = new Pose(28, 12.6, Math.toRadians(0));
    private final Pose clip3 = new Pose(29.7, 14.1, Math.toRadians(0));

    private final Pose clip4 = new Pose(29.7, 15.6, Math.toRadians(0));
    private final Pose clip5 = new Pose(30, 17.1, Math.toRadians(0));
    private final Pose prePush1 = new Pose(47.8, -33, Math.toRadians(0)); //front
    private final Pose helper = new Pose(21.5, -15, Math.toRadians(0)); //like near post
    private final Pose set1 = new Pose(46, -22, Math.toRadians(0)); //left front of samples
    private final Pose observe1 = new Pose(7.7, -33, Math.toRadians(0)); //ob zone
    private final Pose set2 = new Pose(47, -33, Math.toRadians(0));
    private final Pose prePush2 = new Pose(46.5, -38, Math.toRadians(0));
    private final Pose observe2 = new Pose(10.5, -38, Math.toRadians(0));
    private final Pose set3 = new Pose(48, -42, Math.toRadians(0));
    private final Pose prePush3 = new Pose(48, -45, Math.toRadians(0));
    private final Pose observe3 = new Pose(11.3, -45, Math.toRadians(0));
    private final Pose grabPos = new Pose(0.5, -26, Math.toRadians(0));
    private final Pose groundGrab1 = new Pose(21.9161, -13.2278, Math.toRadians(310.966));
    private final Pose deposit1 = new Pose(18.119, -15.7396, Math.toRadians(238.3991));
    private final Pose groundGrab2 = new Pose(23.3654, -22.6956, Math.toRadians(310.2748));
    private final Pose deposit2 = new Pose(17.976, -21.3913, Math.toRadians(229.069));
    private final Pose groundGrab3 = new Pose(22.622, -31.9177, Math.toRadians(306.215));
    private final Pose turn = new Pose (19.033, -31.5586, Math.toRadians(0));
    private final Pose deposit3 = new Pose(19.033, -31.5586, Math.toRadians(195.122));
    //two bar 0.5 and 0.5



    private Follower follower;
    private PathChain barToGround, groundToDepo1, depoToGround1, groundToDepo2, depoToGround2, groundToDepo3, turnPreGrab;
    private PathChain bar1, bar5, grab4, pushPoint1, pushPoint2, pushPoint3, pushPoint4, pushPoint5, pushPoint6, pushPoint7, pushPoint8, pushPoint9, pushPoint10, grab1, grab2, grab3, bar2, bar3, bar4,park;
    Timer opmodeTimer;
    Timer pathTimer;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

//        limelight.pipelineSwitch(0);
        /*
         * Starts polling for data.
         */

        limelight.start();


        twoBarL = hardwareMap.get(Servo.class, "intakeBarL");
        twoBarL.scaleRange(0.425, 0.72);
        twoBarL.setPosition(0.8);//1 - 0.8

        twoBarR = hardwareMap.get(Servo.class, "intakeBarR");
        twoBarR.scaleRange(0.29, 0.585);
        twoBarR.setPosition(0.2);//0 - 0.2

        //DANGER ZONE

        InWrist = hardwareMap.get(Servo.class, "intakeWrist");
        InWrist.scaleRange(0.18, 0.82);
        InWrist.setPosition(wristUp);

        OutWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        //OutWrist.scaleRange(0.25, 0.9);
        OutWrist.setPosition(OutWristInit);


        InElbow = hardwareMap.get(Servo.class, "intakeElbow");

        InElbow.setPosition(InElbowInit);

        OutElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        //OutElbow.scaleRange(0.2, 0.6);
        OutElbow.setPosition(0.47);
        //DANGER ZONE

        rShoulder = hardwareMap.get(Servo.class, "shoulderR");
        rShoulder.scaleRange(0.05, 0.9);
        rShoulder.setPosition(0.83);//0.9

        lShoulder = hardwareMap.get(Servo.class, "shoulderL");
        lShoulder.scaleRange(0.1, 0.95);
        lShoulder.setPosition(0.17);//0.1

        for(int i = 0; i < 5; i++){

        grabMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        grabMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorL.setTargetPosition(50);
        grabMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorL.setPower(0.5);



        grabMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        grabMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        grabMotorR.setTargetPosition(50);
        grabMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorR.setPower(0.5);
        }

        InClaw = hardwareMap.get(Servo.class, "intakeClaw");
        InClaw.scaleRange(0.1, 0.65);
        InClaw.setPosition(InClawInit);

        OutClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        OutClaw.scaleRange(0.4, 0.7);
        OutClaw.setPosition(OutClawInit);

        leftHang = hardwareMap.get(Servo.class, "turnHangL");
        leftHang.setPosition(0.5);
        rightHang = hardwareMap.get(Servo.class, "turnHangR");
        rightHang.setPosition(0.5);




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

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        OutWrist.setPosition(0);
        //rShoulder.setPosition(0.9);
        //lShoulder.setPosition(0.1);
//        OutElbow.setPosition(pickUpSpecElbow);
        setPathState("slides");
        grabMotorR.setTargetPosition(800);
        grabMotorL.setTargetPosition(800);
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
                .addPath(new BezierLine(new Point(observe2), new Point(grabPos)))
                .setLinearHeadingInterpolation(observe2.getHeading(), grabPos.getHeading())
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
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clip3), new Point(grabPos)))
                .setLinearHeadingInterpolation(clip3.getHeading(), grabPos.getHeading())
                .build();
        barToGround = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clip1), new Point(groundGrab1)))
                .setLinearHeadingInterpolation(clip1.getHeading(), groundGrab1.getHeading())
                .build();
        groundToDepo1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(groundGrab1), new Point(deposit1)))
                .setLinearHeadingInterpolation(groundGrab1.getHeading(), deposit1.getHeading())
                .build();
        depoToGround1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(deposit1), new Point(groundGrab2)))
                .setLinearHeadingInterpolation(deposit1.getHeading(), groundGrab2.getHeading())
                .build();
        groundToDepo2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(groundGrab2), new Point(deposit2)))
                .setLinearHeadingInterpolation(groundGrab2.getHeading(), deposit2.getHeading())
                .build();
        depoToGround2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(deposit2), new Point(groundGrab3)))
                .setLinearHeadingInterpolation(deposit2.getHeading(), groundGrab3.getHeading())
                .build();

        groundToDepo3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(groundGrab3), new Point(deposit3)))
                .setLinearHeadingInterpolation(groundGrab3.getHeading(), deposit3.getHeading())
                .build();
        turnPreGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(deposit3), new Point(turn)))
                .setLinearHeadingInterpolation(deposit3.getHeading(), turn.getHeading())
                .build();
        grab4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clip4), new Point(grabPos)))
                .setLinearHeadingInterpolation(clip4.getHeading(), grabPos.getHeading())
                .build();
        bar5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPos), new Point(clip5)))
                .setLinearHeadingInterpolation(grabPos.getHeading(), clip5.getHeading())
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
                case "slides":
                    grabMotorR.setTargetPosition(800);
                    grabMotorL.setTargetPosition(800);
                    if (pathTimer.getElapsedTime() > 1500){
                        setPathState("set wrist");
                    }
                case "set wrist":
                    grabMotorR.setTargetPosition(800);
                    grabMotorL.setTargetPosition(800);
                    if (!follower.isBusy()) {
                        follower.followPath(bar1, true);
                        setPathState("move to bar");
                    }
                    //OutClaw.setPosition(0.98);
                    OutWrist.setPosition(0.175);
                    break;

                case "move to bar":
                        twoBarL.setPosition(0.8);
                        twoBarR.setPosition(0.2);
                        OutWrist.setPosition(0.175);

                    if (pathTimer.getElapsedTime() >= 200) {
                        specStep++;
                        setPathState("clip");
                    }
                    break;

                case "clip":
                    outtakeClawOpen = false;
                    OutClaw.setPosition(0.98);
                    if (pathTimer.getElapsedTime() >= 300) {
                        OutElbow.setPosition(pickUpSpecElbow);
                        specStep++;
                        setPathState("next clip step");
                    }
                    break;
                case "next clip step":

                    if (pathTimer.getElapsedTime() >= 100) {
                        specStep++;
                        setPathState("another clip step");
                    }
                    break;

                case "another clip step":
                    OutElbow.setPosition(1);
                    if (pathTimer.getElapsedTime() >= 100) {
                        specStep++;
                        setPathState("yet another");
                    }
                    break;

                case "yet another":
                    rShoulder.setPosition(0.5);
                    lShoulder.setPosition(0.5);
                    if (pathTimer.getElapsedTime() >= 200) {
                        specStep++;
                        setPathState("two to go");
                    }
                    break;
                case "two to go":
                    grabMotorR.setTargetPosition(1645);
                    grabMotorL.setTargetPosition(1645);
                    if (pathTimer.getElapsedTime() >= 200) {
                        specStep++;
                        setPathState("last clip");
                    }
                    break;
                case "last clip":

                    if (!follower.isBusy())
                    {
                        setPathState("open claw");
                    }
                    break;
                case "open claw":
                    if (pathTimer.getElapsedTime() > 300) {
                        OutClaw.setPosition(clawOpen);
                        outtakeClawOpen = true;
                        grabMotorL.setTargetPosition(0);
                        grabMotorR.setTargetPosition(0);
                        setPathState("grab 1st sample");
                    }
                    break;

                case "grab 1st sample":
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    InWrist.setPosition(0.5);
                    if (!follower.isBusy()) {
                        rShoulder.setPosition(0.84);
                        lShoulder.setPosition(0.16);
                        follower.followPath(pushPoint1, true);
                        //limelight grab
                        setPathState("time");
                    }
                    break;
                case "time":
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    //InClaw.setPosition(clawOpen);
                    InWrist.setPosition(0.5);
                    if (!follower.isBusy())
                    {
                        follower.followPath(pushPoint2, true);
                        setPathState("arm grab ground1");
                    }
                    break;
                case "arm grab ground1":
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    if (!follower.isBusy()) {
//                    if (follower.getPose().roughlyEquals(set1,0.3)) {
                        follower.followPath(pushPoint3, true);//HERE

                        setPathState("split it up1");
                    }
                    break;
                case "split it up1":
                    /*if (pathTimer.getElapsedTime() > 200)
                    {
                        adjustWrist();
                    }
                    InElbow.setPosition(0.3);
                    //InElbow.setPosition(0.15);
                    //InClaw.setPosition(clawClose);*/
                    if (!follower.isBusy())
                    {
                        follower.followPath(pushPoint4, true);
                        setPathState("deposit1");
                    }
                    break;

                case "deposit1":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint5, true);
                        setPathState("case");
                    }
                    break;
                case "case":
                    //InClaw.setPosition(clawOpen);//HEERE
                    if (!follower.isBusy()) {
//                    if (follower.getPose().roughlyEquals(set1,0.3)) {
                        follower.followPath(pushPoint6, true);
                        setPathState("split it up2");
                    }
                    break;

                case "split it up2":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint7, true);
//                        setPathState("grab 2nd sample");
                        setPathState("INTERN");
                    }
                    break;

                case "grab 2nd sample":
                    rShoulder.setPosition(0.84);
                    lShoulder.setPosition(0.16);
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    //InClaw.setPosition(clawOpen);
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint8, true);
                        setPathState("arm grab ground2");
                        //limelight grab
                    }
                    break;
                case "arm grab ground2":
                    /*if (pathTimer.getElapsedTime() > 200)
                    {
                        adjustWrist();
                    }

                    //InElbow.setPosition(0.15);
                    //InClaw.setPosition(clawClose);*/
                    if (!follower.isBusy())
                    {
                        follower.followPath(pushPoint9, true);
                        setPathState("deposit2");
                    }
                    break;

                case "deposit2":
                    InElbow.setPosition(0.3);
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint10, true);
                        setPathState("split it up3");
                    }
                    break;
                case "split it up3":
                    OutClaw.setPosition(clawOpen);
                    rShoulder.setPosition(0.83);
                    lShoulder.setPosition(0.17);
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    if (!follower.isBusy())
                    {
                        setPathState("Grab1");
                    }
                    break;

                case "INTERN":
//                    if(pathTimer.getElapsedTime()>1000)setPathState("Grab1");
                    if(!follower.isBusy())setPathState("Grab1");
                    break;
                    //FULL REFORGE STARTS AT THIS POINT

                case "Grab1":
                    follower.followPath(grab1);
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    rShoulder.setPosition(0.73);
                    lShoulder.setPosition(0.27);
                    grabMotorL.setTargetPosition(50);
                    grabMotorR.setTargetPosition(50);
                    OutWrist.setPosition(0.175);
                    OutElbow.setPosition(0.47);
//                    OutElbow.setPosition(0.25);
                    InElbow.setPosition(0.5);
                    OutClaw.setPosition(clawOpen);
                    setPathState("GrabClose1");
                    break;

                case "GrabClose1":
                    if (!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                        rShoulder.setPosition(0.83);
                        lShoulder.setPosition(0.17);
                        if(pathTimer.getElapsedTime() > 1000)setPathState("toBar1");

                    }
                    break;

                case "toBar1":
                    if (pathTimer.getElapsedTime() > 300) {
                    OutClaw.setPosition(clawClose);}
                    if (pathTimer.getElapsedTime() > 500) {
                        grabMotorR.setTargetPosition(800);
                        grabMotorL.setTargetPosition(800);
                        if (pathTimer.getElapsedTime() > 750){follower.followPath(bar2);
                        setPathState("Bar1Place1");}
                    }
                    break;

                case "Bar1Place1":
                    if(pathTimer.getElapsedTime() > 250){
                    rShoulder.setPosition(0.5);
                    lShoulder.setPosition(0.5);
                    OutElbow.setPosition(1);
                    if (pathTimer.getElapsedTime() > 2000)setPathState("Bar1Place2");
                    }
                    break;

                case "Bar1Place2"://PLACE THE SPECIMEN
                    if(!follower.isBusy()){
                        grabMotorR.setTargetPosition(1645);
                        grabMotorL.setTargetPosition(1645);
                        if(pathTimer.getElapsedTime() > 600){
                            OutClaw.setPosition(clawOpen);
                            setPathState("ToWall2");
                        }
                    }
                    break;

                case "ToWall2":
                    follower.followPath(grab2);
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    rShoulder.setPosition(0.73);
                    lShoulder.setPosition(0.27);
                    grabMotorL.setTargetPosition(50);
                    grabMotorR.setTargetPosition(50);
                    OutWrist.setPosition(0.175);
                    OutElbow.setPosition(0.47);
//                    OutElbow.setPosition(0.25);
                    InElbow.setPosition(0.5);
                    OutClaw.setPosition(clawOpen);
                    setPathState("GrabClose2");
                    break;

                case "GrabClose2":
                    if (!follower.isBusy()) {
                        rShoulder.setPosition(0.83);
                        lShoulder.setPosition(0.17);
                        if(pathTimer.getElapsedTime() > 1000)setPathState("toBar2");

                    }
                    break;

                case "toBar2":
                    if (pathTimer.getElapsedTime() > 300) {
                        OutClaw.setPosition(clawClose);}
                    if (pathTimer.getElapsedTime() > 600) {
                        grabMotorR.setTargetPosition(850);
                        grabMotorL.setTargetPosition(850);
                        if (pathTimer.getElapsedTime() > 750){follower.followPath(bar2);
                            setPathState("Bar2Place1");}
                    }
                    break;
                case "Bar2Place1":
                    if(pathTimer.getElapsedTime() > 250){
                        rShoulder.setPosition(0.5);
                        lShoulder.setPosition(0.5);
                        OutElbow.setPosition(1);
                        if (pathTimer.getElapsedTime() > 2000)setPathState("Bar2Place2");
                    }
                    break;
                case "Bar2Place2"://PLACE THE SPECIMEN
                    if(!follower.isBusy()){
                        grabMotorR.setTargetPosition(1645);
                        grabMotorL.setTargetPosition(1645);
                        if(pathTimer.getElapsedTime() > 650){
                            OutClaw.setPosition(clawOpen);
                            setPathState("ToWall3");
                        }
                    }
                    break;
                case "ToWall3":
                    follower.followPath(grab2);
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    rShoulder.setPosition(0.73);
                    lShoulder.setPosition(0.27);
                    grabMotorL.setTargetPosition(50);
                    grabMotorR.setTargetPosition(50);
                    OutWrist.setPosition(0.175);
                    OutElbow.setPosition(0.47);
//                    OutElbow.setPosition(0.25);
                    InElbow.setPosition(0.5);
                    OutClaw.setPosition(clawOpen);
                    setPathState("GrabClose3");
                    break;
                case "GrabClose3":
                    if (!follower.isBusy()) {
                        rShoulder.setPosition(0.83);
                        lShoulder.setPosition(0.17);
                        if(pathTimer.getElapsedTime() > 1000)setPathState("toBar3");

                    }
                    break;

                case "toBar3":
                    if (pathTimer.getElapsedTime() > 300) {
                        OutClaw.setPosition(clawClose);}
                    if (pathTimer.getElapsedTime() > 600) {
                        grabMotorR.setTargetPosition(850);
                        grabMotorL.setTargetPosition(850);
                        if (pathTimer.getElapsedTime() > 750){follower.followPath(bar3);
                            setPathState("Bar3Place1");}
                    }
                    break;
                case "Bar3Place1":
                    if(pathTimer.getElapsedTime() > 250){
                        rShoulder.setPosition(0.5);
                        lShoulder.setPosition(0.5);
                        OutElbow.setPosition(1);
                        if (pathTimer.getElapsedTime() > 2000)setPathState("Bar3Place2");
                    }
                    break;
                case "Bar3Place2"://PLACE THE SPECIMEN
                    if(!follower.isBusy()){
                        grabMotorR.setTargetPosition(1645);
                        grabMotorL.setTargetPosition(1645);
                        if(pathTimer.getElapsedTime() > 650){
                            OutClaw.setPosition(clawOpen);
                            setPathState("PARKER");
                        }
                    }
                    break;
                case "PARKER":
                    follower.followPath(park);
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    rShoulder.setPosition(0.73);
                    lShoulder.setPosition(0.27);
                    grabMotorL.setTargetPosition(50);
                    grabMotorR.setTargetPosition(50);
                    OutWrist.setPosition(0.175);
                    OutElbow.setPosition(0.47);
//                    OutElbow.setPosition(0.25);
                    InElbow.setPosition(0.5);
                    OutClaw.setPosition(clawOpen);
                    setPathState("Terminate");
                    break;

                case "Terminate":
                    if(!follower.isBusy()){
                        setPathState("Fini ;)");
                    }
                        break;



                /*case "grab 3rd sample":
                    InClaw.setPosition(clawOpen);
                    if (!follower.isBusy()) {
                        follower.followPath(depoToGround2, true);
                        setPathState("arm grab ground3");
                        //limelight grab
                    }
                    break;
                case "arm grab ground3":
                    if (pathTimer.getElapsedTime() > 200)
                    {
                        adjustWrist();
                    }
                    //InElbow.setPosition(0.15);
                    //InClaw.setPosition(clawClose);
                    if (!follower.isBusy())
                    {
                        setPathState("don't hit wall");
                    }
                    break;
                case "don't hit wall":
                    InElbow.setPosition(0.3);
                    twoBarL.setPosition(0.8);
                    twoBarR.setPosition(0.2);
                    if (!follower.isBusy())
                    {
                        setPathState("deposit3");
                    }
                    break;

                case "deposit3":
                    OutClaw.setPosition(clawOpen);
                    if (!follower.isBusy()) {
                        follower.followPath(groundToDepo3, true);
                        setPathState("too many cases");
                    }
                    break;*/
//                case "too many cases":
//                    twoBarL.setPosition(0.8);
//                    twoBarR.setPosition(0.2);
//
//                    outtakeClawOpen = true;
//                    if (!follower.isBusy())
//                    {
//                        OutClaw.setPosition(clawOpen);
//                        setPathState("Grab1");
//                    }
//                    break;
                /*case "turn":
                    if (!follower.isBusy())
                    {
                        follower.followPath(turnPreGrab);
                        setPathState("Grab1");
                    }
                    break;*/
//                case "Grab1": //help meeeee
//                    OutClaw.setPosition(clawOpen);
//                    rShoulder.setPosition(0.83);
//                    lShoulder.setPosition(0.17);
//                    twoBarL.setPosition(0.8);
//                    twoBarR.setPosition(0.2);
//                    rShoulder.setPosition(0.73);
//                    lShoulder.setPosition(0.27);
//                    if (!follower.isBusy()) {
//                        follower.followPath(grab1, true);
//                        setPathState("arm grab");
//                    }
//                    break;
//                    //Do arm pos stuff
//                    //if condition
//                        //path state change
//                case "arm grab":
//                    //arm movement to grab off wall
//                    grabMotorL.setTargetPosition(50);
//                    grabMotorR.setTargetPosition(50);
//
//                    OutWrist.setPosition(0.175);
//                    OutElbow.setPosition(0.25);
//                    InElbow.setPosition(0.5);
//                    OutClaw.setPosition(clawOpen);
//
//                    if (!follower.isBusy()) {
//                        rShoulder.setPosition(0.83);
//                        lShoulder.setPosition(0.17);
//                        setPathState("close claw");
//                    }
//                    break;
//
//                case "close claw":
//                    //arm movement to grab off wall
//                    if (pathTimer.getElapsedTime() > 700) {
//                        OutClaw.setPosition(clawClose);
//                        setPathState("move");
//                    }
//
//                    break;
//
//                case "move":
//                    if (pathTimer.getElapsedTime() > 400) {grabMotorR.setTargetPosition(800);
//                        grabMotorL.setTargetPosition(800);}
//                    if (pathTimer.getElapsedTime() > 1100) {
//                        //OutClaw.setPosition(0.98);
//                        OutWrist.setPosition(0.175);
//                        OutElbow.setPosition(pickUpSpecElbow);
//
//                        follower.followPath(bar2, true);
//                        setPathState("move to bar2");
//                    }
//
//                    break;
//
//                case "move to bar2":
//
//                    /*OutWrist.setPosition(0);
//                    OutElbow.setPosition(pickUpSpecElbow);
//                    if (specElapsedTime >= 2000) {
//                        specStep++;
//                        specStartTime = System.currentTimeMillis();
//
//                    }*/
//                    twoBarL.setPosition(0.8);
//                    twoBarR.setPosition(0.2);
//                    if (!follower.isBusy())
//                    {
//
//                        setPathState("clip2");
//                    }
//                    break;
//
//                case "clip2":
//                    OutClaw.setPosition(0.98);
//                    InElbow.setPosition(0.5);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("next clip step2");
//                    }
//                    break;
//                case "next clip step2" :
//                    //grabMotorR.setTargetPosition(600);
//                    //grabMotorL.setTargetPosition(600);
//                    if (pathTimer.getElapsedTime() >= 300) {
//                        specStep++;
//                        setPathState("another clip step2");
//                    }
//                    break;
//                case "another clip step2":
//                    OutElbow.setPosition(1);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("yet another2");
//                    }
//                    break;
//                case "yet another2":
//                    InElbow.setPosition(0.5);
//                    rShoulder.setPosition(0.5);
//                    lShoulder.setPosition(0.5);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("two to go2");
//                    }
//                    break;
//                case "two to go2":
//                    grabMotorR.setTargetPosition(1645);
//                    grabMotorL.setTargetPosition(1645);
//                    if (pathTimer.getElapsedTime() >= 200)
//                    {
//                        specStep++;
//                        setPathState("last clip2");
//                    }
//                    break;
//                case "last clip2":
//                    OutClaw.setPosition(0.98);
//                    outtakeClawOpen = true;
//                    if (!follower.isBusy())
//                    {
//                        setPathState("open claw2");
//                    }
//                    break;
//
//                case "open claw2":
//                    if (pathTimer.getElapsedTime() > clawWait){
//
//                        OutClaw.setPosition(clawOpen);
//                        twoBarL.setPosition(0.8);
//                        twoBarR.setPosition(0.2);
//                        setPathState("Grab2");
//                    }
//                    break;
//
//                case "Grab2":
//                    InElbow.setPosition(0.5);
//
//                    outtakeClawOpen = true;
//                    if (!follower.isBusy())
//                    {
//                        OutClaw.setPosition(clawOpen);
//                        follower.followPath(grab2, true);
//                        setPathState("arm grab2");
//                        rShoulder.setPosition(0.83);
//                        lShoulder.setPosition(0.17);
//                    }
//                    break;
//                case "arm grab2":
//                    //arm movement to grab off wall
//                    grabMotorL.setTargetPosition(50);
//                    grabMotorR.setTargetPosition(50);
//                    rShoulder.setPosition(0.83);
//                    lShoulder.setPosition(0.17);
//                    OutWrist.setPosition(0.175);
//                    OutElbow.setPosition(0.25);
//                    InElbow.setPosition(0.5);
//                    OutClaw.setPosition(clawOpen);
//                    if (!follower.isBusy())
//                    {
//                        setPathState("close claw2");
//                    }
//                    break;
//
//                case "close claw2":
//                    //arm movement to grab off wall
//                    if (pathTimer.getElapsedTime() > 500) {
//                        OutClaw.setPosition(clawClose);
//                        setPathState("move2");
//                    }
//
//                    break;
//
//                case "move2":
//                     if (pathTimer.getElapsedTime() > 1100)
//                     {
//                         OutClaw.setPosition(0.98);
//                         OutWrist.setPosition(0.175);
//                         OutElbow.setPosition(pickUpSpecElbow);
//                         grabMotorR.setTargetPosition(800);
//                         grabMotorL.setTargetPosition(800);
//                         outtakeClawOpen = false;
//                     }
//
//                    if (!follower.isBusy()) {
//                        follower.followPath(bar3, true);
//
//                        setPathState("move to bar3");
//                    }
//                    break;
//                case "move to bar3":
//
//                   /* OutWrist.setPosition(0);
//                    OutElbow.setPosition(pickUpSpecElbow);
//                    if (specElapsedTime >= 2000) {
//                        specStep++;
//                        specStartTime = System.currentTimeMillis();
//
//                    }*/
//                    twoBarL.setPosition(0.8);
//                    twoBarR.setPosition(0.2);
//                    if (!follower.isBusy())
//                    {
//
//                        setPathState("clip3");
//                    }
//                    break;
//
//                case "clip3":
//                    OutClaw.setPosition(0.98);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("next clip step3");
//                    }
//                    break;
//                case "next clip step3" :
//                    //grabMotorR.setTargetPosition(600);
//                    //grabMotorL.setTargetPosition(600);
//                    if (pathTimer.getElapsedTime() >= 300) {
//                        specStep++;
//                        setPathState("another clip step3");
//                    }
//                    break;
//                case "another clip step3":
//                    OutElbow.setPosition(1);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("yet another3");
//                    }
//                    break;
//                case "yet another3":
//                    rShoulder.setPosition(0.5);
//                    lShoulder.setPosition(0.5);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("two to go3");
//                    }
//                    break;
//                case "two to go3":
//                    grabMotorR.setTargetPosition(1645);
//                    grabMotorL.setTargetPosition(1645);
//                    if (pathTimer.getElapsedTime() >= 200)
//                    {
//                        specStep++;
//                        setPathState("last clip3");
//                    }
//                    break;
//                case "last clip3":
//                    OutClaw.setPosition(0.98);
//                    outtakeClawOpen = true;
//                    if (!follower.isBusy())
//                    {
//                        setPathState("open claw3");
//                    }
//                    break;
//
//                case "open claw3":
//                    if (pathTimer.getElapsedTime() > clawWait){
//
//                        OutClaw.setPosition(clawOpen);
//                        twoBarL.setPosition(0.2);
//                        twoBarR.setPosition(0.2);
//                        setPathState("Grab3");
//                    }
//                    break; //problem
//                case "Grab3":
//                    InElbow.setPosition(0.5);
//
//                    outtakeClawOpen = true;
//                    grabMotorL.setTargetPosition(50);
//                    grabMotorR.setTargetPosition(50);
//                    if (!follower.isBusy())
//                    {
//                        OutClaw.setPosition(clawOpen);
//                        follower.followPath(grab3, true);
//                        setPathState("arm grab3");
//                    }
//                    break;
//                case "arm grab3":
//                    //arm movement to grab off wall
//
//                    grabMotorL.setTargetPosition(0);
//                    grabMotorR.setTargetPosition(0);
//                    rShoulder.setPosition(0.83);
//                    lShoulder.setPosition(0.17);
//                    OutWrist.setPosition(0.175);
//                    OutElbow.setPosition(0.25);
//                    InElbow.setPosition(0.5);
//                    OutClaw.setPosition(clawOpen);
//
//                    if (!follower.isBusy())
//                    {
//                        setPathState("move3");
//                    }
//                    break;
//                case "move3":
//                    if (pathTimer.getElapsedTime() > 1000)
//                    {
//                        OutClaw.setPosition(clawClose);
//                    }
//                    if (pathTimer.getElapsedTime() > 1100)
//                    {
//                        OutClaw.setPosition(0.98);
//                        OutWrist.setPosition(0.175);
//                        OutElbow.setPosition(pickUpSpecElbow);
//                        grabMotorR.setTargetPosition(800);
//                        grabMotorL.setTargetPosition(800);
//                        outtakeClawOpen = false;
//                    }
//
//                    if (!follower.isBusy()) {
//                        follower.followPath(bar4, true);
//                        setPathState("move to bar4");
//                    }
//                    break;
//                case "move to bar4":
//
//                    /*OutWrist.setPosition(0);
//                    OutElbow.setPosition(pickUpSpecElbow);
//                    if (specElapsedTime >= 2000) {
//                        specStep++;
//                        specStartTime = System.currentTimeMillis();
//
//                    }*/
//                    if (!follower.isBusy())
//                    {
//
//                        setPathState("clip4");
//                    }
//                    break;
//
//                case "clip4":
//                    OutClaw.setPosition(0.98);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("next clip step4");
//                    }
//                    break;
//                case "next clip step4" :
//                    //grabMotorR.setTargetPosition(600);
//                    //grabMotorL.setTargetPosition(600);
//                    if (pathTimer.getElapsedTime() >= 300) {
//                        specStep++;
//                        setPathState("another clip step4");
//                    }
//                    break;
//                case "another clip step4":
//                    OutElbow.setPosition(pickUpSpecElbow);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("yet another4");
//                    }
//                    break;
//                case "yet another4":
//                    rShoulder.setPosition(0.5);
//                    lShoulder.setPosition(0.5);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("two to go4");
//                    }
//                    break;
//                case "two to go4":
//                    grabMotorR.setTargetPosition(1645);
//                    grabMotorL.setTargetPosition(1645);
//                    if (pathTimer.getElapsedTime() >= 200)
//                    {
//                        specStep++;
//                        setPathState("last clip4");
//                    }
//                    break;
//                case "last clip4":
//                    OutClaw.setPosition(0.98);
//                    outtakeClawOpen = true;
//                    if (!follower.isBusy())
//                    {
//                        setPathState("open claw4");
//                    }
//                    break;
//
//                case "open claw4":
//                    if (pathTimer.getElapsedTime() > clawWait){
//
//                        OutClaw.setPosition(clawOpen);
//                        setPathState("Grab4");
//                    }
//                    break;
//                case "Grab4":
//                    InElbow.setPosition(0.5);
//
//                    outtakeClawOpen = true;
//                    grabMotorL.setTargetPosition(50);
//                    grabMotorR.setTargetPosition(50);
//                    if (!follower.isBusy())
//                    {
//                        OutClaw.setPosition(clawOpen);
//                        follower.followPath(grab4, true);
//                        setPathState("arm grab4");
//                    }
//                    break;
//                case "arm grab4":
//                    //arm movement to grab off wall
//
//                    grabMotorL.setTargetPosition(0);
//                    grabMotorR.setTargetPosition(0);
//                    rShoulder.setPosition(0.83);
//                    lShoulder.setPosition(0.17);
//                    OutWrist.setPosition(0.175);
//                    OutElbow.setPosition(0.25);
//                    InElbow.setPosition(0.5);
//                    OutClaw.setPosition(clawOpen);
//
//                    if (!follower.isBusy())
//                    {
//                        setPathState("move4");
//                    }
//                    break;
//                case "move4":
//                    if (pathTimer.getElapsedTime() > 1000)
//                    {
//                        OutClaw.setPosition(clawClose);
//                    }
//                    if (pathTimer.getElapsedTime() > 1100)
//                    {
//                        OutClaw.setPosition(0.98);
//                        OutWrist.setPosition(0.175);
//                        OutElbow.setPosition(pickUpSpecElbow);
//                        grabMotorR.setTargetPosition(800);
//                        grabMotorL.setTargetPosition(800);
//                        outtakeClawOpen = false;
//                    }
//
//                    if (!follower.isBusy()) {
//                        follower.followPath(bar5, true);
//
//                        setPathState("move to bar5");
//                    }
//                    break;
//                case "move to bar5":
//
//                    /*OutWrist.setPosition(0);
//                    OutElbow.setPosition(pickUpSpecElbow);
//                    if (specElapsedTime >= 2000) {
//                        specStep++;
//                        specStartTime = System.currentTimeMillis();
//
//                    }*/
//                    if (!follower.isBusy())
//                    {
//
//                        setPathState("clip5");
//                    }
//                    break;
//
//                case "clip5":
//                    OutClaw.setPosition(0.98);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("next clip step5");
//                    }
//                    break;
//                case "next clip step5" :
//                    //grabMotorR.setTargetPosition(600);
//                    //grabMotorL.setTargetPosition(600);
//                    if (pathTimer.getElapsedTime() >= 300) {
//                        specStep++;
//                        setPathState("another clip step5");
//                    }
//                    break;
//                case "another clip step5":
//                    OutElbow.setPosition(pickUpSpecElbow);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("yet another5");
//                    }
//                    break;
//                case "yet another5":
//                    rShoulder.setPosition(0.5);
//                    lShoulder.setPosition(0.5);
//                    if (pathTimer.getElapsedTime() >= 100) {
//                        specStep++;
//                        setPathState("two to go5");
//                    }
//                    break;
//                case "two to go5":
//                    grabMotorR.setTargetPosition(1645);
//                    grabMotorL.setTargetPosition(1645);
//                    if (pathTimer.getElapsedTime() >= 200)
//                    {
//                        specStep++;
//                        setPathState("last clip5");
//                    }
//                    break;
//                case "last clip5":
//                    OutClaw.setPosition(0.98);
//                    outtakeClawOpen = true;
//                    if (!follower.isBusy())
//                    {
//                        setPathState("open claw5");
//                    }
//                    break;
//
//                case "open claw5":
//                    if (pathTimer.getElapsedTime() > clawWait){
//
//                        OutClaw.setPosition(clawOpen);
//                        setPathState("end");
//                    }
//                    break;

            }
        }
        public void setPathState (String pState){
            pathState = pState;
            pathTimer.resetTimer();
        }
    public void adjustWrist() {
        InClaw.setPosition(clawOpen);

        array = result.getPythonOutput();
        angle = array[2];
        rangifiedAngle = angle/300;
        if (rangifiedAngle < 0){
            wristPos = -0.3-rangifiedAngle+0.5;
        }
        else{
            wristPos = 0.3-rangifiedAngle+0.5;
        }
        if (pathTimer.getElapsedTime() > 200)
        {
            InWrist.setPosition(wristPos);
            InElbow.setPosition(0.15);
            InClaw.setPosition(clawClose);
        }
        telemetry.addData("result", result);
        telemetry.addData("angle", angle);
        telemetry.addData("wrist pos", wristPos);
        InWrist.setPosition(wristPos);
        }
    }
