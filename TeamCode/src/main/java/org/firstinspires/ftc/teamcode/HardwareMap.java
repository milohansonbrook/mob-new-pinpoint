package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMap {
    LinearOpMode opmode;
    Servo intake, claw, outtakeWrist, outtakeArmLeft, outtakeArmRight, twoBarL, twoBarR, slurp;
    DcMotor grabMotorL, grabMotorR;

    public HardwareMap(LinearOpMode opmode){
        this.opmode = opmode;
    }

    public void init(){
        intake = this.opmode.hardwareMap.get(Servo.class, "intakeRight");
        claw = this.opmode.hardwareMap.get(Servo.class, "claw");
        outtakeWrist = this.opmode.hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeArmLeft = this.opmode.hardwareMap.get(Servo.class, "outtakeArmLeft");
        outtakeArmRight = this.opmode.hardwareMap.get(Servo.class, "outtakeArmRight");
        twoBarL = this.opmode.hardwareMap.get(Servo.class, "twoBarL");
        twoBarR = this.opmode.hardwareMap.get(Servo.class, "twoBarR");
        slurp = this.opmode.hardwareMap.get(Servo.class, "slurp");
        grabMotorL = this.opmode.hardwareMap.get(DcMotor.class, "grabMotorL");
        grabMotorR = this.opmode.hardwareMap.get(DcMotor.class, "grabMotorR");
    }

}
