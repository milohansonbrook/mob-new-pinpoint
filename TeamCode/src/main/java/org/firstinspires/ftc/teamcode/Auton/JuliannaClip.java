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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous (name="Auton Specimen Red", group = "Autonomous")
public class JuliannaClip extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private PathChain scorePreload, park;
    private PathChain pushThings, prePush, grab1, grabScore1, back1, grabScore2, back2, grabScore3;
    private final Pose startPose = new Pose(134, 95, Math.toRadians(180));
    private final Pose clipPose = new Pose(105, 75, Math.toRadians(180));
    private final Pose pushPose = new Pose(88, 117, Math.toRadians(180));
    private final Pose pushBack1 = new Pose(130, 120, Math.toRadians(180));
    private final Pose pushBack2 = new Pose(130, 133, Math.toRadians(180));
    private final Pose pushBack3 = new Pose(88, 133, Math.toRadians(180));
    private final Pose pushBack4 = new Pose(130,142, Math.toRadians(180));
    private final Pose grabPose = new Pose(133, 120, Math.toRadians(180));
    private final Pose clipPose2 = new Pose(100, 76, Math.toRadians(180));
    private final Pose clipPose3 = new Pose(100, 74, Math.toRadians(180));
    private final Pose clipPose4 = new Pose(100, 72, Math.toRadians(180));
    private final Pose end = new Pose(134, 127, Math.toRadians(180));
    private Servo Rshoulder, turnGrabber, openGrabber, slurp, turnSlurp;
    private DcMotor Lslide, Rslide, out;

    public void buildPaths()
    {
        Constants.setConstants(FConstants.class, LConstants.class);
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(clipPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), clipPose.getHeading())
                .build();
        //clip();

        prePush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(clipPose), new Point(120, 114), new Point(pushPose)))
                .setLinearHeadingInterpolation(clipPose.getHeading(), pushPose.getHeading())
                .build();
        pushThings = follower.pathBuilder()
                .addBezierLine(new Point(pushPose), new Point(pushBack1))
                .setLinearHeadingInterpolation(pushPose.getHeading(), pushBack1.getHeading())
                .addBezierCurve(new Point(pushBack1), new Point(52, 120), new Point(110, 143), new Point(pushBack2))
                .setLinearHeadingInterpolation(pushBack1.getHeading(), pushBack2.getHeading())
                .addBezierLine(new Point(pushBack2), new Point(pushBack3))
                .setLinearHeadingInterpolation(pushBack2.getHeading(), pushBack3.getHeading())
                .addBezierCurve(new Point(pushBack3), new Point(74, 140), new Point(pushBack4))
                .setLinearHeadingInterpolation(pushBack3.getHeading(), pushBack4.getHeading())
                .build();
        grab1 = follower.pathBuilder()
                .addBezierLine(new Point(pushBack4), new Point(grabPose))
                .setLinearHeadingInterpolation(pushBack4.getHeading(), grabPose.getHeading())
                .build();
        //grab();
        grabScore1 = follower.pathBuilder()
                .addBezierLine(new Point(grabPose), new Point(clipPose2))
                .setLinearHeadingInterpolation(grabPose.getHeading(), clipPose2.getHeading())
                .build();
        //clip();
        back1 = follower.pathBuilder()
                .addBezierLine(new Point(clipPose2), new Point(grabPose))
                .setLinearHeadingInterpolation(clipPose2.getHeading(), grabPose.getHeading())
                .build();
        //grab();
        grabScore2 = follower.pathBuilder()
                .addBezierLine(new Point(grabPose), new Point(clipPose3))
                .setLinearHeadingInterpolation(grabPose.getHeading(), clipPose3.getHeading())
                .build();
        //clip();
        back2 = follower.pathBuilder()
                .addBezierLine(new Point(clipPose3), new Point(grabPose))
                .setLinearHeadingInterpolation(clipPose3.getHeading(), grabPose.getHeading())
                .build();
        //grab();
        grabScore3 = follower.pathBuilder()
                .addBezierLine(new Point(grabPose), new Point(clipPose4))
                .setLinearHeadingInterpolation(grabPose.getHeading(), clipPose4.getHeading())
                .build();
        //clip();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clipPose4), new Point(end)))
                .setLinearHeadingInterpolation(clipPose4.getHeading(), end.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(prePush,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushThings,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grab1,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabScore1,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(back1,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabScore2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(back2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy())
                {
                    follower.followPath(grabScore3,true);
                    setPathState(9);
                }
            case 9:
                if(!follower.isBusy())
                {
                    follower.followPath(park, true);
                    setPathState(10);
                }
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void grab()
    {
        //shoulder back, claw open, claw close, shoulder forward, claw rotate, slide up, claw rotate, slide down, claw open
        turnSlurp.setPosition(0.85);
        //Lshoulder.setPosition(0.45);
        Rshoulder.setPosition(0.55);
        turnGrabber.setPosition(0.75);
        openGrabber.setPosition(0.25);

        openGrabber.setPosition(0.05);

        turnGrabber.setPosition(0.2);
        //Lshoulder.setPosition(0.7);
        Rshoulder.setPosition(0.3);

    }
    public void clip() {
        turnSlurp.setPosition(0.85);
        openGrabber.setPosition(0.05);
        //Lshoulder.setPosition(0.7);
        Rshoulder.setPosition(0.3);
        if (Lslide.getCurrentPosition() < 300 || Rslide.getCurrentPosition() < 300)//MAKE SURE THE MOTORS ARE GOING THE RIGHT WAY BEFORE RUNNING THIS!!!!!
        {
            while (Lslide.getCurrentPosition() < 300 || Rslide.getCurrentPosition() < 300) {
                Lslide.setPower(0.75);
                Rslide.setPower(0.75);
            }
        }
        turnGrabber.setPosition(0.25);

        //Lshoulder.setPosition(0.9);
        Rshoulder.setPosition(0.1);
        turnGrabber.setPosition(0.05);
        openGrabber.setPosition(0.25);
        if (Lslide.getCurrentPosition() > 10 || Rslide.getCurrentPosition() > 10)//MAKE SURE THE MOTORS ARE GOING THE RIGHT WAY BEFORE RUNNING THIS!!!!!
        {
            while (Lslide.getCurrentPosition() > 10 || Rslide.getCurrentPosition() > 10) {
                Lslide.setPower(-0.75);
                Rslide.setPower(-0.75);
            }
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
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}





