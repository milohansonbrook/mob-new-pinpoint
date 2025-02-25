package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "die")
public class die extends LinearOpMode {
    private Servo twoBarR;
    private Servo twoBarL;
    public static double LPose = 0.425;
    public static double RPose = 0.29;
    @Override
    public void runOpMode() throws InterruptedException {
        twoBarR = hardwareMap.get(Servo.class, "twoBarR");
        twoBarL = hardwareMap.get(Servo.class, "twoBarL");
        waitForStart();
        while (opModeIsActive()) {
            twoBarR.setPosition(RPose);
            twoBarL.setPosition(LPose);
        }
    }
}
