package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class zeroServos extends LinearOpMode {
    Servo one;
    Servo two;

    @Override
    public void runOpMode() throws InterruptedException {
        one = hardwareMap.get(Servo.class, "one");
        one.scaleRange(0.48, 0.535);
        one.setPosition(0);

        two = hardwareMap.get(Servo.class, "two");
        two.scaleRange(0.465, 0.52);
        two.setPosition(1);
        waitForStart();
        while( opModeIsActive()){

            if (gamepad1.right_trigger > 0.1) {
                one.setPosition(one.getPosition() - 0.01*gamepad1.right_trigger);
                two.setPosition(two.getPosition() + 0.01*gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                one.setPosition(one.getPosition() + 0.01*gamepad1.left_trigger);
                two.setPosition(two.getPosition() - 0.01*gamepad1.left_trigger);

        }
        }
    }
}


