package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
@Config
public class AlexBond2 extends LinearOpMode {
    public double barInterval = 0.007;
    public double wristInterval = 0.01;
    public double clawInterval = 0.01;
    public double shoulderInterval = 0.01;
    public double elbowDown = 1;
    public double elbowHunting = 0.85;
    public double elbowUp = 0;
    public double wristHoriz = 0.17;
    long transferStartTime = 0;
    int transferStep = 0;
    // left Servo (two bar)
    Servo intakeBarL; //0C
    // right Servo
    Servo intakeBarR; //0E
    // moves entire claw up and down
    Servo intakeElbow; //1E
    // moves outtake claw up and down
    Servo shoulderR; //2C
    Servo shoulderL; //2E
    // turns claw left and right
    Servo intakeWrist; //4E
    // opens and closes claw
    Servo intakeClaw; //5E
    Servo outtakeElbow; //1C
    Servo outtakeWrist;//4C
    Servo outtakeClaw; //5C

    // moves vertical slides up and down
    DcMotor slideMotorR; //3E
    DcMotor slideMotorL; //3C
    boolean transferActive;
    boolean transferComplete;
    boolean outtakeClawOpen = true;
    boolean intakeClawOpen = true;
    boolean hunting;
    boolean huntingLast;
    boolean aLast;
    boolean b2Last;
    boolean yLast;
    boolean halfSpeed;
    double drivePower;
    public static int wait1 = 300;
    public static int wait2 = 250;
    public static int wait3 = 10;
    public static int wait4 = 400;
    public static int wait5 = 500;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        intakeBarL = hardwareMap.get(Servo.class, "intakeBarL");
        intakeBarL.scaleRange(0.35, 0.67);
        intakeBarL.setPosition(1);
        intakeBarR = hardwareMap.get(Servo.class, "intakeBarR");
        intakeBarR.scaleRange(0.33, 0.65);
        intakeBarR.setPosition(0);

        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeClaw.setPosition(0.5);
        outtakeClaw.scaleRange(0.4, 0.7);
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.setPosition(0.55);
        outtakeWrist.scaleRange(0.25, 0.9);
        outtakeElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        outtakeElbow.setPosition(0.5);
        outtakeElbow.scaleRange(0.2, 0.6);
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        shoulderL.setPosition(0.1);
        shoulderL.scaleRange(0.1, 0.95);
        shoulderR = hardwareMap.get(Servo.class, "shoulderR");
        shoulderR.setPosition(0.9);
        shoulderR.scaleRange(0.05, 0.9);

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setPosition(0.5);
        intakeClaw.scaleRange(0.39, 0.65);
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeWrist.setPosition(0.5);
        intakeWrist.scaleRange(0.18, 0.82);
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeElbow.setPosition(0.6);
        intakeElbow.scaleRange(0.23, 0.76);

        slideMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setPower(1);

        slideMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setTargetPosition(0);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(1);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        halfSpeed = false;
//pre init code above________________________________________________________________________________________________________________________________

        waitForStart();

//TWO BAR ADJUSTMENT________________________________________________________________________
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            if (gamepad2.b && !b2Last) {
                halfSpeed = !halfSpeed;
            }
            drivePower = halfSpeed ? 0.25 : 1;
            b2Last = gamepad2.b;

            //drive
            follower.setTeleOpMovementVectors(-gamepad2.left_stick_y * drivePower, -gamepad2.left_stick_x * drivePower, -gamepad2.right_stick_x * drivePower, true);
            follower.update();

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            if (gamepad1.left_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() + barInterval * gamepad1.left_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() - barInterval * gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() - barInterval * gamepad1.right_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() + barInterval * gamepad1.right_trigger);
            }

            if (gamepad1.left_stick_y > 0.1) {
                //slideMotorL.setTargetPosition(slideMotorL.getCurrentPosition() + barInterval * gamepad1.left_stick_y);
            }
//Code Booleans!!!________________________________________________________________________

            if (intakeClawOpen) {
                intakeClaw.setPosition(0);
            }
            else {
                intakeClaw.setPosition(1);
            }

            if (outtakeClawOpen) {
                outtakeClaw.setPosition(0);
            }
            else {
                outtakeClaw.setPosition(1);
            }

            // Toggle hunting mode when dpad_left is pressed
            if (gamepad1.dpad_left && !huntingLast) {
                hunting = !hunting;
            }

            huntingLast = gamepad1.dpad_left;

            if (hunting) {
                intakeElbow.setPosition(elbowHunting);
                intakeClawOpen = true;

                if (gamepad1.dpad_left) intakeWrist.setPosition(0.5); //double press good vert
                else if (gamepad1.dpad_right) intakeWrist.setPosition(0); //good horizontal
                else if (gamepad1.dpad_up) intakeWrist.setPosition(0.75); //good 45 right
                else if (gamepad1.dpad_down) intakeWrist.setPosition(0.25); //good 45 left
            } else {
                if (gamepad1.dpad_right) {
                    intakeElbow.setPosition(elbowDown);
                    intakeClawOpen = false;
                }

                if (gamepad1.x && !transferActive) {
                    slideMotorL.setTargetPosition(0);
                    slideMotorR.setTargetPosition(0);
                    intakeClawOpen = false;
                    intakeElbow.setPosition(elbowUp);
                    intakeWrist.setPosition(1);
                    outtakeElbow.setPosition(0.1);
                    outtakeWrist.setPosition(0.42);
                    shoulderL.setPosition(0.2);
                    shoulderR.setPosition(0.8);
                    transferStep = 0;
                    transferActive = true;
                    transferComplete = false;
                    transferStartTime = System.currentTimeMillis();
                }

                if (gamepad1.b) {
                    slideMotorL.setTargetPosition(0);
                    slideMotorR.setTargetPosition(0);
                    intakeClawOpen = true;
                    outtakeClawOpen = true;
                    shoulderL.setPosition(0.15);
                    shoulderR.setPosition(0.85);
                    intakeElbow.setPosition(elbowUp);
                    transferStep = 0;
                    transferActive = false;
                    transferComplete = true;
                }
            }



//Code Actions!!!________________________________________________________________________

            if (gamepad1.right_bumper) intakeWrist.setPosition(intakeWrist.getPosition() + 0.005);
            if (gamepad1.left_bumper) intakeWrist.setPosition(intakeWrist.getPosition() - 0.005);

            if (gamepad1.a && !aLast) {
                intakeClawOpen = !intakeClawOpen;
            }
            aLast = gamepad1.a;

            if (gamepad1.y && !yLast) {
                outtakeClawOpen = !outtakeClawOpen;
            }
            yLast = gamepad1.y;

//SAMPLE TRANSFER SEQUENCE BELOW
                if (transferActive && !transferComplete) {
                    long transferElapsedTime = System.currentTimeMillis() - transferStartTime;
                    switch (transferStep) {
                        //point elbow down when over sample
                        case 0:
                            slideMotorL.setTargetPosition(0);
                            slideMotorR.setTargetPosition(0);
                            intakeBarL.setPosition(0.83);
                            intakeBarR.setPosition(0.17);
                            if (transferElapsedTime >= wait1) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;

                        case 1:
                            shoulderL.setPosition(0.15);
                            shoulderR.setPosition(0.85);
                            outtakeWrist.setPosition(0.42);
                            outtakeElbow.setPosition(0.17);
                            if (transferElapsedTime >= wait2) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 2:
                            outtakeClaw.setPosition(1);
                            outtakeClawOpen = false;
                            if (transferElapsedTime >= wait3) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 3:
                            intakeClaw.setPosition(0);
                            intakeClawOpen = true;
                            intakeBarL.setPosition(0.75);
                            intakeBarR.setPosition(0.25);
                            if (transferElapsedTime >= wait4) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 4:
                            shoulderL.setPosition(0.9);
                            shoulderR.setPosition(0.1);
                            slideMotorL.setTargetPosition(2500);
                            slideMotorR.setTargetPosition(2500);
                            outtakeWrist.setPosition(0);
                            outtakeElbow.setPosition(0.5);
                            transferComplete = true;
                            transferActive = false;
                            if (transferElapsedTime >= wait5) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                    }
                }
            telemetry.addData("outtake claw pos", outtakeClaw.getPosition());
            telemetry.addData("outtake wrist pos", outtakeWrist.getPosition());
            telemetry.addData("outtake elbow pos", outtakeElbow.getPosition());
            telemetry.addData("intake claw pos", intakeClaw.getPosition());
            telemetry.addData("intake wrist pos", intakeWrist.getPosition());
            telemetry.addData("intake elbow pos", intakeElbow.getPosition());
            telemetry.addData("shoulderL pos", shoulderL.getPosition());
            telemetry.addData("shoulderR pos", shoulderR.getPosition());
            telemetry.addData("left slide pos", slideMotorL.getCurrentPosition());
            telemetry.addData("right slide pos", slideMotorR.getCurrentPosition());
            telemetry.addData("two bar L pos", intakeBarL.getPosition());
            telemetry.addData("two bar R pos", intakeBarR.getPosition());
            telemetry.addData("transfer active?", transferActive);
            telemetry.addData("transfer step", transferStep);
            telemetry.update();
        }
/*
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

 */
                /*
                elbow down
                close claw
                twist wrist and move elbow back to up facing position
                */
           /*
                switch (intakeSequence) {
                    case 0:
                        intakeClaw.setPosition(0);


                }
            }

                }

            */
    }
}







