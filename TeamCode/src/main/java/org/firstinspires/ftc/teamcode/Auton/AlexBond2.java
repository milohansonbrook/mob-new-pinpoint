package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class AlexBond2 extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {

        intakeBarL = hardwareMap.get(Servo.class, "intakeBarL");
        intakeBarL.scaleRange(0.35, 0.65);
        intakeBarL.setPosition(0);
        intakeBarR = hardwareMap.get(Servo.class, "intakeBarR");
        intakeBarR.scaleRange(0.35, 0.65);
        intakeBarR.setPosition(1);

        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeWrist.setPosition(0.5);
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setPosition(0.5);
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeElbow.setPosition(0.5);


        //pre init code above________________________________________________________________________________________________________________________________
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.right_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() + 0.007*gamepad1.right_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() - 0.007*gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() - 0.007*gamepad1.left_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() + 0.007*gamepad1.left_trigger);
            }

            if(gamepad1.dpad_left) {
                intakeWrist.setPosition(intakeWrist.getPosition() + 0.005);
            }
            if(gamepad1.dpad_right) {
                intakeWrist.setPosition(intakeWrist.getPosition() - 0.005);
            }
            if(gamepad1.dpad_up) {
                intakeClaw.setPosition(intakeClaw.getPosition() + 0.005);
            }
            if(gamepad1.dpad_down) {
                intakeClaw.setPosition(intakeClaw.getPosition() - 0.005);
            }
        }

    }
}