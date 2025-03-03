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

    //Waits
    public static int clawWait = 1000;

    //technical poses

    public static double armMidPosL = 0.7;
    public static double armMidPosR = (1 - armMidPosL);
    public static double barDown = 0.5;
    public static double wristUp = 1;
    public static double wristStraight = 0.4;
    public static double backForWall = 0.0;
    public static double OutWristInit = 0.0;
    public static double InElbowInit = 0.0;
    public static double OutElbowInit = 0.0;
    public static int slideWall = 50;
    public static double OutClawInit = 0.0;
    public static double InClawInit = 0.0;
    public static double clawClose = 1;
    public static double clawOpen = 0;
    public static int slideClipPose1 = 250;
    public static int slideClipPose2 = 650;
    public static double startSlurp = 0.35;

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
        InWrist.scaleRange(0, 1);
        InWrist.setPosition(wristUp);

        OutWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        OutWrist.scaleRange(0,1);
        OutWrist.setPosition(OutWristInit);

        InElbow = hardwareMap.get(Servo.class, "intakeElbow");
        InElbow.scaleRange(0,1);
        InElbow.setPosition(InElbowInit);

        OutElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        OutElbow.scaleRange(0,1);
        OutElbow.setPosition(OutElbowInit);

        rShoulder = hardwareMap.get(Servo.class, "shoulderR");
        rShoulder.scaleRange(0, 1);
        rShoulder.setPosition(armMidPosR);

        lShoulder = hardwareMap.get(Servo.class, "shoulderL");
        lShoulder.scaleRange(0, 1);
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
        InClaw.scaleRange(0.525, 0.64);
        InClaw.setPosition(InClawInit);

        OutClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        OutClaw.scaleRange(0,1);
        OutClaw.setPosition(OutClawInit);



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

                    setPathState("move to bar");
                    break;

                case "move to bar":
                    if (pathTimer.getElapsedTime() > 300) {
                        twoBarL.setPosition(0.75);
                        twoBarR.setPosition(0.25);
                        setBarPose(1);
                        OutWrist.setPosition(wristStraight);
                        setVertSlide(slideClipPose1);
                        twoBarL.setPosition(1);
                        twoBarR.setPosition(0);
                        follower.followPath(bar1, true);
                        setPathState("clip");
                    }
                    break;

                case "clip":
                    if (pathTimer.getElapsedTime() > 2000){
                        setVertSlide(slideClipPose2);
                        setPathState("open claw");
                    }
                    break;

                case "open claw":
                    if (pathTimer.getElapsedTime() > clawWait){
                        OutWrist.setPosition(wristUp);
                        OutClaw.setPosition(clawOpen);
                        twoBarL.setPosition(0.75);
                        twoBarR.setPosition(0.25);
                        setBarPose(barDown);
                        setPathState("move set1");
                    }
                    break;

                case "move set1":
                    setVertSlide(slideWall);
                    twoBarL.setPosition(1);
                    twoBarR.setPosition(0);
                    follower.followPath(pushPoint1, true);
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
                        follower.followPath(grab1, true);
                        setPathState("arm grab");
                    }
                    break;
                case "arm grab":
                    //arm movement to grab off wall

                    rShoulder.setPosition(backForWall);
                    lShoulder.setPosition(1-backForWall);
                    OutClaw.setPosition(clawClose);
                    setPathState("Clip2");
                    break;
                case "Clip2":
                    if (!follower.isBusy())
                    {
                        //prepare arm movements
                        twoBarL.setPosition(0.75);
                        twoBarR.setPosition(0.25);
                        setBarPose(1);
                        OutWrist.setPosition(wristStraight);
                        setVertSlide(slideClipPose1);
                        twoBarL.setPosition(1);
                        twoBarR.setPosition(0);
                        follower.followPath(bar2, true);
                        setPathState("arm clip");
                    }
                    break;
                case "arm clip":
                    //clip movement
                    setVertSlide(slideClipPose2);
                    setPathState("Grab2");
                    break;
                case "Grab2":
                    if (!follower.isBusy())
                    {
                        follower.followPath(grab2, true);
                        setPathState("arm grab2");
                    }
                    break;
                case "arm grab2":
                    //use limelight to adjust grabber
                    //arm movement to grab off wall
                    setPathState("Clip3");
                    break;
                case "Clip3":
                    if (!follower.isBusy())
                    {
                        //prepare arm movements
                        follower.followPath(bar3, true);
                        setPathState("arm clip2");
                    }
                    break;
                case "arm clip2":
                    //clip movement
                    setPathState("Grab3");
                    break;
                case "Grab3":
                    if (!follower.isBusy())
                    {
                        follower.followPath(grab3, true);
                        setPathState("arm grab3");
                    }
                    break;
                case "arm grab3":
                    //use limelight to adjust grabber
                    //arm movement to grab off wall
                    setPathState("Clip4");
                    break;
                case "Clip4":
                    if (!follower.isBusy())
                    {
                        //prepare arm movements
                        follower.followPath(bar4, true);
                        setPathState("arm clip3");
                    }
                    break;
                case "arm clip3":
                    //clip movement
                    setPathState("end");
                    break;
            }
        }
        public void setPathState (String pState){
            pathState = pState;
            pathTimer.resetTimer();
        }
}