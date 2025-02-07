package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class megaMacro extends LinearOpMode {
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
    boolean b2Last;
    boolean halfSpeed;
    double drivePower;
    long sampleSequenceStartTime = 0;
    long specimenSequenceStartTime = 0;
    int sampleSequenceStep = 0;
    int specimenSequenceStep = 0;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        clawState = true;

        turnSlurp = hardwareMap.get(Servo.class, "turnSlurp");
        turnSlurp.scaleRange(slurpLowerBound, slurpUpperBound);
        turnSlurp.setPosition(1);

        twoBarL = hardwareMap.get(Servo.class, "twoBarL");
        twoBarL.scaleRange(0.425, 0.69);
        twoBarL.setPosition(1);

        twoBarR = hardwareMap.get(Servo.class, "twoBarR");
        twoBarR.scaleRange(0.31, 0.575);
        twoBarR.setPosition(0);

        turnClaw = hardwareMap.get(Servo.class, "turnClaw");
        turnClaw.scaleRange(0, 1);
        turnClaw.setPosition(wristMidPos);

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
        claw.setPosition(0);

        slurp = hardwareMap.get(CRServo.class, "slurp");

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        halfSpeed = false;

//These are the actions that take effect when you press the start button_______________________________________________________________________________________________________________________________
        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            if (gamepad1.a){
                claw.setPosition(1);
                grabMotorL.setTargetPosition(clawHeight);
                grabMotorR.setTargetPosition(clawHeight);
            }

            telemetry.addData("here: ", turnSlurp.getPosition());
        }
    }
}
