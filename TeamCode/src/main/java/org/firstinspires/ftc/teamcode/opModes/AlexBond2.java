package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    boolean aLast;
    boolean yLast;

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
        intakeClaw.scaleRange(0.4, 0.65);
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeWrist.setPosition(0.5);
        intakeWrist.scaleRange(0.18, 0.82);
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeElbow.setPosition(0.6);
        intakeElbow.scaleRange(0.23, 0.76);

        slideMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setPower(1);

        slideMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setTargetPosition(0);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(1);


//pre init code above________________________________________________________________________________________________________________________________

        waitForStart();

//TWO BAR ADJUSTMENT________________________________________________________________________

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() + barInterval * gamepad1.left_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() - barInterval * gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() - barInterval * gamepad1.right_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() + barInterval * gamepad1.right_trigger);
            }

//Code Booleans!!!________________________________________________________________________

            if (intakeClawOpen) intakeClaw.setPosition(0);
            else intakeClaw.setPosition(1);

            if (outtakeClawOpen) outtakeClaw.setPosition(0);
            else outtakeClaw.setPosition(1);

//Code Actions!!!________________________________________________________________________
            if (gamepad1.dpad_left) intakeElbow.setPosition(elbowHunting);
            if (gamepad1.dpad_right) intakeElbow.setPosition(elbowDown);

            if (gamepad1.right_bumper) intakeWrist.setPosition(intakeWrist.getPosition() + 0.005);
            if (gamepad1.left_bumper) intakeWrist.setPosition(intakeWrist.getPosition() - 0.005);

            if (gamepad1.a && !aLast) {
                intakeClawOpen = !intakeClawOpen;
            }
            aLast = gamepad1.a;
// Sequence Starts and Stops_______________________________________________________________
            if (gamepad1.dpad_up && !transferActive) {
                intakeElbow.setPosition(elbowUp);
                intakeWrist.setPosition(1);
                outtakeElbow.setPosition(0.1);
                outtakeWrist.setPosition(0.42);
                shoulderL.setPosition(0.25);
                shoulderR.setPosition(0.75);
                transferStep = 0;
                transferActive = true;
                transferComplete = false;
                transferStartTime = System.currentTimeMillis();
            }
            if (gamepad1.dpad_down) {
                intakeClawOpen = true;
                outtakeClawOpen = true;
                shoulderL.setPosition(0.25);
                shoulderR.setPosition(0.75);
                intakeElbow.setPosition(elbowUp);
                transferStep = 0;
                transferActive = false;
                transferComplete = true;
            }

//SAMPLE TRANSFER SEQUENCE BELOW
                if (transferActive && !transferComplete) {
                    long transferElapsedTime = System.currentTimeMillis() - transferStartTime;
                    switch (transferStep) {
                        //point elbow down when over sample
                        case 0:
                            intakeBarL.setPosition(0.83);
                            intakeBarR.setPosition(0.17);
                            if (transferElapsedTime >= 2000) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;

                        case 1:
                            shoulderL.setPosition(0.15);
                            shoulderR.setPosition(0.85);
                            outtakeWrist.setPosition(0.42);
                            outtakeElbow.setPosition(0.17);
                            if (transferElapsedTime >= 2000) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;

                        case 2:
                            outtakeClawOpen = false;
                            if (transferElapsedTime >= 2000) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                        case 3:
                            intakeClawOpen = true;
                            if (transferElapsedTime >= 2000) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                        case 4:
                            shoulderL.setPosition(0.8);
                            shoulderR.setPosition(0.2);
                            slideMotorL.setTargetPosition(20);
                            slideMotorR.setTargetPosition(20);
                            transferComplete = true;
                            transferActive = false;
                            break;
                    }
                }
            
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
        telemetry.addData("outtake claw pos", outtakeClaw);
        telemetry.addData("outtake wrist pos", outtakeWrist);
        telemetry.addData("outtake elbow pos", outtakeElbow);
        telemetry.addData("intake claw pos", intakeClaw);
        telemetry.addData("intake wrist pos", intakeWrist);
        telemetry.addData("intake elbow pos", intakeElbow);
        telemetry.addData("intake shoulderL pos", shoulderL);
        telemetry.addData("intake shoulderR pos", shoulderR);
        telemetry.addData("left slide pos", slideMotorL);
        telemetry.addData("right slide pos", slideMotorR);
        telemetry.addData("two bar L pos", intakeBarL);
        telemetry.addData("two bar R pos", intakeBarR);
        telemetry.addData("transfer active?", transferActive);
        telemetry.addData("transfer step", transferStep);

        telemetry.update();
    }
}







