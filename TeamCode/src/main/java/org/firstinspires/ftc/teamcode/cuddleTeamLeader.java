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
    public static double slurpLowerBound = 0.21;
    public static double slurpUpperBound = 0.4;
    public static int clawHeight = 1000;
    public static double armMidPosL = 0.59;
    public static double armMidPosR = (1 - armMidPosL);
    public static double wristMidPos = 0.5;
    boolean intakeDown;
    boolean aLast;
    boolean clawState;
    boolean armSequenceActive;
    boolean armSequenceComplete;
    boolean yLast;
    boolean jiggle;
    long sequenceStartTime = 0;
    int armSequenceStep = 0;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        clawState = true;

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
        grabMotorL.setPower(1);

        grabMotorR = hardwareMap.get(DcMotor.class, "grabMotorR");
        grabMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        grabMotorR.setTargetPosition(0);
        grabMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorR.setPower(1);

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

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            // SLURPY
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

            //ARM MOTOR CODE
            if (gamepad1.dpad_right) {
                grabMotorL.setTargetPosition(clawHeight);
                grabMotorR.setTargetPosition(clawHeight);
                outtakeArmRight.setPosition(0.8);
                outtakeArmLeft.setPosition(0.2);
                outtakeWrist.setPosition(0.4);
            } else if (gamepad1.dpad_left) {
                grabMotorL.setTargetPosition(0);
                grabMotorR.setTargetPosition(0);
                outtakeArmRight.setPosition(armMidPosR);
                outtakeArmLeft.setPosition(armMidPosL);
                outtakeWrist.setPosition(wristMidPos);
                claw.setPosition(0);
            }

            //two bar code
            if (gamepad1.right_trigger > 0.1) {
                twoBarL.setPosition(twoBarL.getPosition() - 0.04*gamepad1.right_trigger);
                twoBarR.setPosition(twoBarR.getPosition() + 0.04*gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                twoBarL.setPosition(twoBarL.getPosition() + 0.04*gamepad1.left_trigger);
                twoBarR.setPosition(twoBarR.getPosition() - 0.04*gamepad1.left_trigger);
            }


            // potential specimen code
            if (gamepad1.x) {
                grabMotorL.setTargetPosition(500);
                grabMotorR.setTargetPosition(500);
            }
            if (gamepad1.left_stick_button) {
                grabMotorL.setTargetPosition(100);
                grabMotorR.setTargetPosition(100);
                outtakeArmRight.setPosition(0.8);
                outtakeArmLeft.setPosition(0.2);
                outtakeWrist.setPosition(0.4);
            }
            if (gamepad1.right_stick_button) {
                grabMotorL.setTargetPosition(700);
                grabMotorR.setTargetPosition(700);
                outtakeArmRight.setPosition(0.2);
                outtakeArmLeft.setPosition(0.8);
                outtakeWrist.setPosition(0.2);
            }

            // ARM TOGGLE: Start or reset sequence
            if (gamepad1.dpad_up && !armSequenceActive) {
                armSequenceActive = true;
                armSequenceComplete = false;
                clawState = true;
                armSequenceStep = 0;
                intakeRight.setPosition(1);
                outtakeArmRight.setPosition(armMidPosR);
                outtakeArmLeft.setPosition(armMidPosL);
                outtakeWrist.setPosition(wristMidPos);
                claw.setPosition(0);
                slurp.setPower(1);
                sequenceStartTime = System.currentTimeMillis();
            }
            if (gamepad1.dpad_down) {
                armSequenceActive = false;
                armSequenceComplete = true;
                clawState = true;
                armSequenceStep = 0;
                intakeRight.setPosition(1);
                outtakeArmRight.setPosition(armMidPosR);
                outtakeArmLeft.setPosition(armMidPosL);
                outtakeWrist.setPosition(wristMidPos);
                claw.setPosition(0);
            }

            // Execute arm sequence
            if (armSequenceActive && !armSequenceComplete) {
                long elapsedTime = System.currentTimeMillis() - sequenceStartTime;
                switch (armSequenceStep) {
                    case 0:
                        intakeDown = false;
                        jiggle = true;
                        slurp.setPower(-1);
                        if (elapsedTime >= 500) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        intakeDown = false;
                        jiggle = false;
                        if (elapsedTime >= 700) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 2:
                        intakeDown = false;
                        jiggle = true;
                        grabMotorL.setTargetPosition(0);
                        grabMotorR.setTargetPosition(0);
                        if (elapsedTime >= 500) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        jiggle = true;
                        if (elapsedTime >= 250) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        jiggle = true;
                        clawState = false;
                        slurp.setPower(0);
                        if (elapsedTime >= 500) {
                            armSequenceStep++;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 5:
                        outtakeArmRight.setPosition(1);
                        outtakeArmLeft.setPosition(0);
                        outtakeWrist.setPosition(0);
                        armSequenceComplete = true;
                        armSequenceActive = false;
                        break;
                }
            }

            if (clawState) claw.setPosition(0);
            else claw.setPosition(1);

            if (intakeDown && !jiggle) intakeRight.setPosition(0);
            else if (intakeDown && jiggle) intakeRight.setPosition(0.3);
            else if (!intakeDown && !jiggle) intakeRight.setPosition(1);
            else intakeRight.setPosition(0.5);

            double forward = gamepad2.left_stick_y;   // Forward/backward movement
            double strafe = gamepad2.left_stick_x;     // Left/right movement
            double rotate = gamepad2.right_stick_x;    // Rotation (turning)

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