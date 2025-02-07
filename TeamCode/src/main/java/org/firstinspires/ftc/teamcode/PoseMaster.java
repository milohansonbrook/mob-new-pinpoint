package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
@Config
public class PoseMaster extends LinearOpMode {
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
    //These are our position variables, they are servo and motor positions that can be altered in the FTC driver station.
//if you have a position that you'll need to be editing often, you should make it a variable and put it in here to quickly test new values.
//Notice that these are public variables, that's how driver station accesses them___________________________________________________________________________________________________________________
    public static double slurpLowerBound = 0.21;
    public static double slurpUpperBound = 0.4;
    public static int clawHeight = 860;
    public static double armMidPosL = 0.81;
    public static double armMidPosR = (1 - armMidPosL);
    public static double wristMidPos = 0.5;

    //these are variables we change within the code, mainly true/false variables or counting variables__________________________________________________________________________________________________
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
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    //These are the actions that take place when you press the Init button_____________________________________________________________________________________________________________
    @Override
    public void runOpMode() throws InterruptedException {
        clawState = true;

        turnSlurp = hardwareMap.get(Servo.class, "turnSlurp");

        twoBarL = hardwareMap.get(Servo.class, "twoBarL");

        twoBarR = hardwareMap.get(Servo.class, "twoBarR");

        turnClaw = hardwareMap.get(Servo.class, "turnClaw");

        rShoulder = hardwareMap.get(Servo.class, "rShoulder");

        lShoulder = hardwareMap.get(Servo.class, "lShoulder");

        grabMotorL = hardwareMap.get(DcMotor.class, "grabMotorL");

        grabMotorR = hardwareMap.get(DcMotor.class, "grabMotorR");

        claw = hardwareMap.get(Servo.class, "claw");

        slurp = hardwareMap.get(CRServo.class, "slurp");

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

//These are the actions that take effect when you press the start button_______________________________________________________________________________________________________________________________
        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {

            follower.update();

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

// Add telemetry for debugging______________________________________________________________________________________________________________________________________________________________________________________________________________
            telemetry.addData("Left grab motor pos", grabMotorL.getCurrentPosition());
            telemetry.addData("Right grab motor pos", grabMotorL.getCurrentPosition());
            telemetry.addData("Sequence Step", sampleSequenceStep);
            telemetry.addData("Sequence Active", sampleSequenceActive);
            telemetry.addData("Sequence Complete", sampleSequenceComplete);
            telemetry.addData("Claw State", clawState ? "Open" : "Closed");
            telemetry.addData("turnSlurp", turnSlurp.getPosition());
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("turnClaw", turnClaw.getPosition());
            telemetry.addData("right shoulder", rShoulder.getPosition());
            telemetry.addData("left shoulder", lShoulder.getPosition());
            telemetry.addData("twoBarL", twoBarL.getPosition());
            telemetry.addData("twoBarR", twoBarR.getPosition());
            telemetry.addData("grabMotorL", grabMotorL.getCurrentPosition());
            telemetry.addData("grabMotorR", grabMotorR.getCurrentPosition());

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}

