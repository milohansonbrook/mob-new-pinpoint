package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class die extends LinearOpMode {
    Servo two;

    @Override
    public void runOpMode() throws InterruptedException {
        two = hardwareMap.get(Servo.class, "intakeElbow");
        waitForStart();
        while (opModeIsActive())
        {
            two.setPosition(0.5);
        }
    }
}