
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
@Config
public class cuddleTeamLeader extends LinearOpMode {
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

        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {

            if (gamepad2.b && !b2Last) {
                halfSpeed = !halfSpeed;
            }
            drivePower = halfSpeed ? 0.25 : 1;
            b2Last = gamepad2.b;

            follower.setTeleOpMovementVectors(-gamepad2.left_stick_y * drivePower, -gamepad2.left_stick_x * drivePower, -gamepad2.right_stick_x * drivePower, true);
            follower.update();

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

// SLURPY___________________________________________________________________________________________
            if (gamepad1.right_bumper) {
                slurp.setPower(1);
            } else if (gamepad1.left_bumper) {
                slurp.setPower(-1);
            } else {
                slurp.setPower(0);
            }

            // Toggle intake with A button
            if (gamepad1.a && !aLast) {
                intakeDown = !intakeDown;
            }

            aLast = gamepad1.a;
            jiggle = gamepad1.b;

            if (gamepad1.y && !yLast) {
                clawState = !clawState;
            }
            yLast = gamepad1.y;

//two bar code______________________________________________________________________________________
            if (gamepad1.right_trigger > 0.1) {
                twoBarL.setPosition(twoBarL.getPosition() - 0.04*gamepad1.right_trigger);
                twoBarR.setPosition(twoBarR.getPosition() + 0.04*gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                twoBarL.setPosition(twoBarL.getPosition() + 0.04*gamepad1.left_trigger);
                twoBarR.setPosition(twoBarR.getPosition() - 0.04*gamepad1.left_trigger);
            }
//Free Buttons______________________________________________________________________________________
            if (gamepad1.guide) {

            }
            if (gamepad1.right_stick_button) {

            }
            if (gamepad1.left_stick_button) {

            }
//specimen code_____________________________________________________________________________________
            if (gamepad1.x) {
                grabMotorL.setTargetPosition(650);
                grabMotorR.setTargetPosition(650);
            }

            // Specimen sequence code
            if (gamepad1.dpad_left) {
                sampleSeqActive = false;
                sampleSeqComplete = true;
                clawState = true;
                sampleSeqStep = 0;
                grabMotorL.setTargetPosition(0);
                grabMotorR.setTargetPosition(0);
                rShoulder.setPosition(0.55);
                lShoulder.setPosition(0.45);
                turnClaw.setPosition(0.75);
            }
            if (gamepad1.dpad_right) {
                sampleSeqActive = true;
                sampleSeqComplete = false;
                clawState = false;
                sampleSeqStep = 0;
                sampleSeqStartTime = System.currentTimeMillis();
            }

            //execute sample sequence
            if (sampleSeqActive && !sampleSeqComplete) {
                long sampleElapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (sampleSeqStep) {
                    case 0:
                        clawState = false;
                        if (sampleElapsedTime >= 300){
                            sampleSeqStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 1:
                        grabMotorL.setTargetPosition(375);
                        grabMotorR.setTargetPosition(375);
                        if (sampleElapsedTime >= 500){
                            sampleSeqStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 2:
                        rShoulder.setPosition(armMidPosR - 0.24);
                        lShoulder.setPosition(armMidPosL + 0.24);
                        turnClaw.setPosition(0.35);
                        armSequenceComplete = true;
                        armSequenceActive = false;
                        break;
                }
            }

// ARM SEQUENCE CODE________________________________________________________________________________
            if (gamepad1.dpad_down) {
                armSequenceActive = false;
                armSequenceComplete = true;
                sampleSeqActive = false;
                sampleSeqComplete = true;
                clawState = true;
                armSequenceStep = 0;
                turnSlurp.setPosition(1);
                rShoulder.setPosition(armMidPosR);
                lShoulder.setPosition(armMidPosL);
                turnClaw.setPosition(wristMidPos);
                grabMotorL.setTargetPosition(0);
                grabMotorR.setTargetPosition(0);
                claw.setPosition(0);
            }

            // ARM TOGGLE: Start or reset sequence
            if (gamepad1.dpad_up && !armSequenceActive) {
                armSequenceActive = true;
                armSequenceComplete = false;
                clawState = true;
                armSequenceStep = 0;
                turnSlurp.setPosition(1);
                rShoulder.setPosition(armMidPosR);
                lShoulder.setPosition(armMidPosL);
                turnClaw.setPosition(wristMidPos);
                claw.setPosition(0);
                slurp.setPower(1);
                sequenceStartTime = System.currentTimeMillis();
            }

            // Execute arm sequence
            if (armSequenceActive && !armSequenceComplete) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (armSequenceStep) {
                    case 0:

                        //slurping.... FLIP UP, GO in,
                        intakeDown = false;
                        twoBarR.setPosition(0);
                        twoBarL.setPosition(1);
                        slurp.setPower(-1);
                        grabMotorL.setTargetPosition(0);
                        grabMotorR.setTargetPosition(0);
                        if (elapsedTime >= 1250) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 1:
                        //closing claw
                        clawState = false;
                        slurp.setPower(-1);
                        if (elapsedTime >= 500) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 2:
                        slurp.setPower(0);
                        grabMotorL.setTargetPosition(clawHeight);
                        grabMotorR.setTargetPosition(clawHeight);
                        if (elapsedTime >= 750) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        rShoulder.setPosition(1);
                        lShoulder.setPosition(0);
                        turnClaw.setPosition(0.55);
                        armSequenceComplete = true;
                        armSequenceActive = false;
                        if (elapsedTime >= 250) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        if (elapsedTime >= 500) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 5:
                        break;
                }
            }

//__________________________________________________________________________________________________

            if (clawState) claw.setPosition(0);
            else claw.setPosition(1);

            //JIGGLE CODE
            if (intakeDown && !jiggle) turnSlurp.setPosition(0);
            else if (intakeDown && jiggle) turnSlurp.setPosition(0.3);
            else if (!intakeDown && !jiggle) turnSlurp.setPosition(1);
            else turnSlurp.setPosition(0.5);

            // Add telemetry for debugging
            telemetry.addData("Left grab motor pos", grabMotorL.getCurrentPosition());
            telemetry.addData("Right grab motor pos", grabMotorL.getCurrentPosition());
            telemetry.addData("Sequence Step", armSequenceStep);
            telemetry.addData("Sequence Active", armSequenceActive);
            telemetry.addData("Sequence Complete", armSequenceComplete);
            telemetry.addData("Claw State", clawState ? "Open" : "Closed");
            telemetry.update();
        }
    }
}
