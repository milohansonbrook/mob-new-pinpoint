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
    Servo intakeRight;
    Servo claw;
    Servo outtakeWrist;
    Servo outtakeArmRight;
    Servo outtakeArmLeft;
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
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeRight.scaleRange(slurpLowerBound, slurpUpperBound);
        intakeRight.setPosition(1);

        twoBarL = hardwareMap.get(Servo.class, "twoBarL");
        twoBarL.scaleRange(0.425, 0.69);
        twoBarL.setPosition(1);

        twoBarR = hardwareMap.get(Servo.class, "twoBarR");
        twoBarR.scaleRange(0.31, 0.575);
        twoBarR.setPosition(0);

        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.scaleRange(0, 1);
        outtakeWrist.setPosition(wristMidPos);

        outtakeArmRight = hardwareMap.get(Servo.class, "outtakeArmRight");
        outtakeArmRight.scaleRange(0, 1);
        outtakeArmRight.setPosition(armMidPosR);

        outtakeArmLeft = hardwareMap.get(Servo.class, "outtakeArmLeft");
        outtakeArmLeft.scaleRange(0, 1);
        outtakeArmLeft.setPosition(armMidPosL);

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
        claw.setPosition(0);

        slurp = hardwareMap.get(CRServo.class, "slurp");


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("inTakeRight", intakeRight.getPosition());
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("outtakeWrist", outtakeWrist.getPosition());
            telemetry.addData("outtakeArmRight", outtakeArmRight.getPosition());
            telemetry.addData("outtakeArmLeft", outtakeArmLeft.getPosition());
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
