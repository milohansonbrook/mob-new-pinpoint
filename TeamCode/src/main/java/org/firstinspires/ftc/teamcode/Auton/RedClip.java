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

@Autonomous(name = "AlexClip", group = "Autonomous")
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

    //technical poses poses
    public static double slurpLowerBound = 0.195;
    public static double slurpUpperBound = 0.65;
    public static double armMidPosL = 0.81;
    public static double armMidPosR = (1 - armMidPosL);
    public static double wristMidPos = 0.5;
    public static double clawClose = 1;

    //drive poses
    public static double clipPoseX;
    public static double clipPoseY;
    public static double clipPoseHeading;


    String pathState = "init";
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose clipPose = new Pose(23.8, 13.1, Math.toRadians(0));
    private final Pose prePush1 = new Pose(47.8, -32, Math.toRadians(0));
    private final Pose helper = new Pose(21.5, -11, Math.toRadians(0));
    private final Pose set1 = new Pose(46, -22, Math.toRadians(0));
    private final Pose observe1 = new Pose(7.7, -30, Math.toRadians(0));
    private final Pose set2 = new Pose(47, -30, Math.toRadians(0));
    private final Pose prePush2 = new Pose(46.5, -40.6, Math.toRadians(0));
    private final Pose observe2 = new Pose(10.5, -38.3, Math.toRadians(0));
    private final Pose set3 = new Pose(47.3, -41, Math.toRadians(0));
    private final Pose prePush3 = new Pose(46.7, -46, Math.toRadians(0));
    private final Pose observe3 = new Pose(11.3, -45, Math.toRadians(0));
    private final Pose grabPos = new Pose(1.8, -37.5, Math.toRadians(0));


    private Follower follower;
    private PathChain bar1, pushThings, otherBar;
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
        wrist.setPosition(0.5);

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
        setPathState("move to bar");
    }

    public void buildPaths() {
        // Path for scoring preload
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        bar1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(clipPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), clipPose.getHeading())
                .build();
        pushThings  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clipPose), new Point(helper)))
                .setLinearHeadingInterpolation(clipPose.getHeading(), helper.getHeading())
                .addBezierLine(new Point(helper), new Point(set1))
                .setLinearHeadingInterpolation(helper.getHeading(), set1.getHeading())
                .addBezierLine(new Point(set1), new Point(prePush1))
                .setLinearHeadingInterpolation(set1.getHeading(), prePush1.getHeading())
                .addBezierLine(new Point(prePush1), new Point(observe1))
                .setLinearHeadingInterpolation(prePush1.getHeading(), observe1.getHeading())
                .addBezierLine(new Point(observe1), new Point(set2))
                .setLinearHeadingInterpolation(observe1.getHeading(), set2.getHeading())
                .addBezierLine(new Point(set2), new Point(prePush2))
                .setLinearHeadingInterpolation(set2.getHeading(), prePush2.getHeading())
                .addBezierLine(new Point(prePush2), new Point(observe2))
                .setLinearHeadingInterpolation(prePush2.getHeading(), observe2.getHeading())
                .addBezierLine(new Point(observe2), new Point(set3))
                .setLinearHeadingInterpolation(observe2.getHeading(), set3.getHeading())
                .addBezierLine(new Point(set3), new Point(prePush3))
                .setLinearHeadingInterpolation(set3.getHeading(), prePush3.getHeading())
                .addBezierLine(new Point(prePush3), new Point(observe3))
                .setLinearHeadingInterpolation(prePush3.getHeading(), observe3.getHeading())
                .addBezierLine(new Point(observe3), new Point(grabPos))
                .setLinearHeadingInterpolation(observe3.getHeading(), grabPos.getHeading())
                .build();
        otherBar = follower.pathBuilder()
                .addPath(new BezierLine(new Point(), new Point()))
                .build();
    }

        public void autonomousPathUpdate() {
            switch (pathState) {
                case "move to bar":
                    turnSlurp.setPosition(0.5);
                    follower.followPath(bar1, true);
                    setPathState("clip");
                    break;

                case "clip":
//                    if (!follower.isBusy()){
//
//                    }
                    break;

            }
        }
        public void setPathState (String pState){
            pathState = pState;
            pathTimer.resetTimer();
        }
}
