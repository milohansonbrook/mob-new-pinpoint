package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class MotorAndServoPoses extends LinearOpMode {
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
    private Telemetry telemetry;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);


    @Override
    public void runOpMode() throws InterruptedException {

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


        waitForStart();
        while (opModeIsActive()) {
            follower.update();
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
