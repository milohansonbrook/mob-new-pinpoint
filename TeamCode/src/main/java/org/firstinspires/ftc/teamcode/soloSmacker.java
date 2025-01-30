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
public class soloSmacker extends LinearOpMode {
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

            if (gamepad1.left_stick_button && !stickB1Last) {
                halfSpeed = !halfSpeed;
            }
            drivePower = halfSpeed ? 0.25 : 1;
            stickB1Last = gamepad2.left_stick_button;

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * drivePower, -gamepad1.left_stick_x * drivePower, -gamepad1.right_stick_x * drivePower, true);
            follower.update();

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

// SLURPY_______________________________________________________________________________________________________________________________________________________________________________________________________
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

//two bar code__________________________________________________________________________________________________________________________________________________________________________________________________
            if (gamepad1.right_trigger > 0.1) {
                twoBarL.setPosition(twoBarL.getPosition() - 0.04*gamepad1.right_trigger);
                twoBarR.setPosition(twoBarR.getPosition() + 0.04*gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                twoBarL.setPosition(twoBarL.getPosition() + 0.04*gamepad1.left_trigger);
                twoBarR.setPosition(twoBarR.getPosition() - 0.04*gamepad1.left_trigger);
            }
//Free Buttons__________________________________________________________________________________________________________________________________________________________________________________________________
            if (gamepad1.guide) {

            }
            if (gamepad1.right_stick_button) {

            }
//specimen code_________________________________________________________________________________________________________________________________________________________________________________________________

            // Specimen sequence buttons______________
            //x sends slides up to place on bar
            if (gamepad1.x) {
                grabMotorL.setTargetPosition(650);
                grabMotorR.setTargetPosition(650);
            }

            //dpad left resets specimen sequence
            if (gamepad1.dpad_left) {
                clawState = true;
                specimenSequenceStep = 0;
                grabMotorL.setTargetPosition(0);
                grabMotorR.setTargetPosition(0);
                rShoulder.setPosition(0.55);
                lShoulder.setPosition(0.45);
                turnClaw.setPosition(0.75);
                //makes sure that the code for any other sequence is killed
                specimenSequenceActive = false;
                specimenSequenceComplete = true;
                sampleSequenceActive = false;
                sampleSequenceComplete = true;
            }

            //dpad right initiates specimen sequence
            if (gamepad1.dpad_right) {
                specimenSequenceActive = true;
                specimenSequenceComplete = false;
                clawState = false;
                specimenSequenceStep = 0;
                specimenSequenceStartTime = System.currentTimeMillis();
            }

            //execute specimen sequence_________________________________________________________________
            if (specimenSequenceActive && !specimenSequenceComplete) {
                long specimenElapsedTime = System.currentTimeMillis() - specimenSequenceStartTime;
                switch (specimenSequenceStep) {
                    case 0:
                        //close claw
                        clawState = false;
                        //wait 0.3 seconds
                        if (specimenElapsedTime >= 300){
                            specimenSequenceStep++;
                            specimenSequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 1:
                        //send motors up to take specimen off wall
                        grabMotorL.setTargetPosition(375);
                        grabMotorR.setTargetPosition(375);
                        if (specimenElapsedTime >= 400){
                            specimenSequenceStep++;
                            specimenSequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 2:
                        //send arm to placing position then wait so the specimen doesn't flip
                        rShoulder.setPosition(armMidPosR - 0.24);
                        lShoulder.setPosition(armMidPosL + 0.24);
                        if (specimenElapsedTime >= 200){
                            specimenSequenceStep++;
                            specimenSequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 3:
                        //send wrist to placing position
                        turnClaw.setPosition(0.35);

                        //declare that the sequence has finished
                        sampleSequenceComplete = true;
                        sampleSequenceActive = false;
                        break;
                }
            }

// sample sequence code____________________________________________________________________________________________________________________________________________________________________________________________

            //reset sample sequence
            if (gamepad1.dpad_down) {
                clawState = true;
                sampleSequenceStep = 0;
                turnSlurp.setPosition(1);
                rShoulder.setPosition(armMidPosR);
                lShoulder.setPosition(armMidPosL);
                turnClaw.setPosition(wristMidPos);
                grabMotorL.setTargetPosition(0);
                grabMotorR.setTargetPosition(0);
                claw.setPosition(0);
                //makes sure that the code for any other sequence is killed
                sampleSequenceActive = false;
                sampleSequenceComplete = true;
                specimenSequenceActive = false;
                specimenSequenceComplete = true;
            }

            // initiate sample sequence
            if (gamepad1.dpad_up && !sampleSequenceActive) {
                sampleSequenceActive = true;
                sampleSequenceComplete = false;
                clawState = true;
                sampleSequenceStep = 0;
                turnSlurp.setPosition(1);
                rShoulder.setPosition(armMidPosR);
                lShoulder.setPosition(armMidPosL);
                turnClaw.setPosition(wristMidPos);
                claw.setPosition(0);
                slurp.setPower(1);
                sampleSequenceStartTime = System.currentTimeMillis();
            }

            // Execute arm sequence__________________________________________________________
            if (sampleSequenceActive && !sampleSequenceComplete) {
                long sampleElapsedTime = System.currentTimeMillis() - sampleSequenceStartTime;
                switch (sampleSequenceStep) {
                    case 0:
                        //slurping.... FLIP UP, GO in, do whatever is needed to push sample to the correct position
                        intakeDown = false;
                        twoBarR.setPosition(0);
                        twoBarL.setPosition(1);
                        slurp.setPower(-1);
                        grabMotorL.setTargetPosition(0);
                        grabMotorR.setTargetPosition(0);
                        if (sampleElapsedTime >= 1250) {
                            sampleSequenceStep++;
                            sampleSequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 1:
                        //closing claw on sample
                        clawState = false;
                        slurp.setPower(-1);
                        if (sampleElapsedTime >= 500) {
                            sampleSequenceStep++;
                            sampleSequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 2:
                        slurp.setPower(0);
                        grabMotorL.setTargetPosition(clawHeight);
                        grabMotorR.setTargetPosition(clawHeight);
                        if (sampleElapsedTime >= 750) {
                            sampleSequenceStep++;
                            sampleSequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        rShoulder.setPosition(1);
                        lShoulder.setPosition(0);
                        turnClaw.setPosition(0.55);
                        sampleSequenceComplete = true;
                        sampleSequenceActive = false;
                        if (sampleElapsedTime >= 250) {
                            sampleSequenceStep++;
                            sampleSequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        if (sampleElapsedTime >= 500) {
                            sampleSequenceStep++;
                            sampleSequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 5:
                        break;
                }
            }

//claw code and jiggle code______________________________________________________________________________________________________________________________________________________________________________________________________________

            if (clawState) claw.setPosition(0);
            else claw.setPosition(1);

            //JIGGLE CODE
            if (intakeDown && !jiggle) turnSlurp.setPosition(0);
            else if (intakeDown && jiggle) turnSlurp.setPosition(0.5);
            else if (!intakeDown && !jiggle) turnSlurp.setPosition(1);
            else turnSlurp.setPosition(1);

// Add telemetry for debugging______________________________________________________________________________________________________________________________________________________________________________________________________________
            telemetry.addData("Left grab motor pos", grabMotorL.getCurrentPosition());
            telemetry.addData("Right grab motor pos", grabMotorL.getCurrentPosition());
            telemetry.addData("Sequence Step", sampleSequenceStep);
            telemetry.addData("Sequence Active", sampleSequenceActive);
            telemetry.addData("Sequence Complete", sampleSequenceComplete);
            telemetry.addData("Claw State", clawState ? "Open" : "Closed");
            telemetry.update();
        }
    }
}
