package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class AlexBond2 extends LinearOpMode {
    public double barInterval = 0.007;
    public double wristInterval = 0.005;
    public double clawInterval = 0.005;
    public double elbowDown = 0.75;
    public double elbowHunting = 0.70;
    public double elbowUp = 0.23;
    public double wristHoriz = 0.17;

    double transferSequenceStartTime = System.currentTimeMillis();
    int transferSequenceStep = 0;

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

    // wheels
    DcMotor leftFront; //0C
    DcMotor leftBack; //1C
    DcMotor rightFront; //0E
    DcMotor rightBack; //1E
    // moves vertical slides up and down
    DcMotor slideMotorR; //3E
    DcMotor slideMotorL; //3C


    boolean transferSequenceActive = true;
    boolean transferSequenceCompleted = false;
    int intakeSequence = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeBarL = hardwareMap.get(Servo.class, "intakeBarL");
        intakeBarL.scaleRange(0.35, 0.65);
        intakeBarL.setPosition(1);
        intakeBarR = hardwareMap.get(Servo.class, "intakeBarR");
        intakeBarR.scaleRange(0.35, 0.65);
        intakeBarR.setPosition(0);

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setPosition(0.5);
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeWrist.setPosition(0.5);
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeElbow.setPosition(0.5);

        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeClaw.setPosition(0.5);
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.setPosition(0.5);
        outtakeElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        outtakeElbow.setPosition(0.5);
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        shoulderL.setPosition(0.5);
        shoulderR = hardwareMap.get(Servo.class, "shoulderR");
        shoulderR.setPosition(0.5);


        //pre init code above________________________________________________________________________________________________________________________________
        waitForStart();

//TWO BAR ADJUSTMENT________________________________________________________________________

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() + barInterval * gamepad1.right_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() - barInterval * gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() - barInterval * gamepad1.left_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() + barInterval * gamepad1.left_trigger);
            }

//INTAKE ADJUSTMENT________________________________________________________________________
            if (gamepad1.dpad_left) {
                intakeWrist.setPosition(intakeWrist.getPosition() + wristInterval);
            }
            if (gamepad1.dpad_right) {
                intakeWrist.setPosition(intakeWrist.getPosition() - wristInterval);
            }
            if (gamepad1.dpad_up) {
                intakeClaw.setPosition(intakeClaw.getPosition() + clawInterval);
            }
            if (gamepad1.dpad_down) {
                intakeClaw.setPosition(intakeClaw.getPosition() - clawInterval);
            }
            else if (gamepad1.x) {
                // down facing elbow position
                intakeElbow.setPosition(elbowDown);
            } else if (gamepad1.b) {
                // up facing elbow position
                intakeElbow.setPosition(elbowUp);
            }
            if (gamepad1.a) {
             // claw open
                intakeSequence = 0;

            }
//OUTTAKE ADJUSTMENT________________________________________________________________________


            //TRANSFER SEQUENCE BELOW


            if (transferSequenceActive && !transferSequenceCompleted) {

                long transferElapsedTime = (long) (System.currentTimeMillis() - transferSequenceStartTime);
                switch (transferSequenceStep) {

                    //point elbow down
                    case 0:
                    intakeElbow.setPosition(elbowHunting);
                        if (transferElapsedTime >= 10000){
                            transferSequenceStep++;
                            transferSequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    //point elbow up
                    case 1:
                        intakeClaw.setPosition(0.67);
                        if (transferElapsedTime >= 10000){
                            transferSequenceStep++;
                            transferSequenceStartTime = System.currentTimeMillis();
                        }
                        break;

                    case 2:

                        intakeWrist.setPosition(wristHoriz);
                        intakeElbow.setPosition(elbowUp);
                        break;

                    //case 2:




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
    }
}





