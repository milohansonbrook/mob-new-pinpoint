package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "freak")
public class freakIt extends LinearOpMode {
    private DcMotor slideMotorL;
    private DcMotor slideMotorR;

    @Override
    public void runOpMode() throws InterruptedException {
        slideMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
