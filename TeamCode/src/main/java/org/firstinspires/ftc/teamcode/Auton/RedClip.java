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

@Autonomous(name = "clippy clip", group = "Autonomous")
@Config
public class RedClip extends OpMode {

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

    //Waits
    public static int clawWait = 1000;

    //technical poses
    public static double slurpLowerBound = 0.19;
    public static double slurpUpperBound = 0.65;
    public static double armMidPosL = 0.7;
    public static double armMidPosR = (1 - armMidPosL);
    public static double barDown = 0.5;
    public static double wristUp = 1;
    public static double wristStraight = 0.4;
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
    private PathChain bar1, pushPoint1, pushPoint2, pushPoint3, pushPoint4, pushPoint5, pushPoint6, pushPoint7, pushPoint8, pushPoint9, pushPoint10;
    Timer opmodeTimer;
    Timer pathTimer;

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
        claw.setPosition(clawClose);

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
                case "set slurp":
                    turnSlurp.setPosition(startSlurp);
                    setPathState("move to bar");
                    break;

                case "move to bar":
                    if (pathTimer.getElapsedTime() > 300) {
                        setBarPose(1);
                        wrist.setPosition(wristStraight);
                        setVertSlide(slideClipPose1);
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
                        claw.setPosition(clawOpen);
                        wrist.setPosition(wristUp);
                        setBarPose(barDown);
                        setPathState("move set");
                    }
                    break;

                case "move set":
                    setVertSlide(0);
                    follower.followPath(pushPoint1, true);
                    follower.followPath(pushPoint2, true);
                    setPathState("yo");
                    break;

                case "yo":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint3, true);
                        follower.followPath(pushPoint4, true);
                        setPathState("yoyo");
                    }
                    break;

                case "yoyo":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint5, true);
                        follower.followPath(pushPoint6, true);
                        setPathState("yoyoyo");
                    }
                    break;

                case "yoyoyo":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint7, true);
                        follower.followPath(pushPoint8, true);
                        setPathState("yoyoyoyo");
                    }
                    break;

                case "yoyoyoyo":
                    if (!follower.isBusy()) {
                        follower.followPath(pushPoint9, true);
                        follower.followPath(pushPoint10, true);
                        setPathState("yoyoyoyoyo");
                    }
                    break;
            }
        }
        public void setPathState (String pState){
            pathState = pState;
            pathTimer.resetTimer();
        }
}
