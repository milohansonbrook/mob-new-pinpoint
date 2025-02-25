package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class die extends LinearOpMode {
    Servo one;
    Servo two;

    @Override
    public void runOpMode() throws InterruptedException {
        one = hardwareMap.get(Servo.class, "shoulderR");
        two = hardwareMap.get(Servo.class, "shoulderL");
        waitForStart();
        while (opModeIsActive())
        {
            one.setPosition(1);
            two.setPosition(0);
        }
    }
}