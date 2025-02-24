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
    public double elbowUp = 0.23;
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


    boolean intakeSequenceActive = true;
    boolean intakeSequenceCompleted = false;
    int intakeSequence = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeBarL = hardwareMap.get(Servo.class, "intakeBarL");
        intakeBarL.scaleRange(0.35, 0.65);
        intakeBarL.setPosition(1);
        intakeBarR = hardwareMap.get(Servo.class, "intakeBarR");
        intakeBarR.scaleRange(0.35, 0.65);
        intakeBarR.setPosition(0);

        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeWrist.setPosition(0.5);
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setPosition(0.5);
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeElbow.setPosition(0.5);


        //pre init code above________________________________________________________________________________________________________________________________
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() + barInterval * gamepad1.right_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() - barInterval * gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() - barInterval * gamepad1.left_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() + barInterval * gamepad1.left_trigger);
            }

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
//
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
            double intakeSequenceStartTime = System.currentTimeMillis();


            if (intakeSequenceActive && !intakeSequenceCompleted) {
                long specimenElapsedTime = (long) (System.currentTimeMillis() - intakeSequenceStartTime);
                intakeElbow.setPosition(elbowDown);

                if (specimenElapsedTime >= 1250)
                {
                    intakeElbow.setPosition(elbowUp);
                }


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





