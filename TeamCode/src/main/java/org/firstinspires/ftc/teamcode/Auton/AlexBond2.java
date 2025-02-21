package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class AlexBond2 extends LinearOpMode {
    Servo one;
    Servo two;
    Servo oneIntakeArm;
    Servo oneOuttakeArm;
    Servo twoIntakeArm;
    Servo twoOuttakeArm;
    Servo intakeWrist;
    Servo intakeClaw;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor grabMotorR;
    DcMotor grabMotorL;

    @Override
    public void runOpMode() throws InterruptedException {

        one = hardwareMap.get(Servo.class, "one");
        one.scaleRange(0.48, 0.585);
        one.setPosition(0);

        two = hardwareMap.get(Servo.class, "one");
        two.scaleRange(0.465, 0.52);
        two.setPosition(1);
        waitForStart();
        while( opModeIsActive()){
            if(gamepad1.right_trigger > 0.1) {
                one.setPosition(one.getPosition());
           }
        }

    }
}