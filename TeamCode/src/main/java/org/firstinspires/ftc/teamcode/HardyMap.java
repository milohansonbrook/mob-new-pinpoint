package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HardyMap {

    //define opmode
    public LinearOpMode opMode;

    //define hardware
    public Servo intake;
    public Servo outtakeWrist;
    public Servo outtakeArmRight;
    public Servo outtakeArmLeft;
    public DcMotor slurpMotor;
    public Servo claw;
    public CRServo slurp;

    public HardyMap(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(){
        intake = this.opMode.hardwareMap.get(Servo.class, "intakeRight");
        outtakeWrist = this.opMode.hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeArmRight = this.opMode.hardwareMap.get(Servo.class, "outtakeArmRight");
        outtakeArmLeft = this.opMode.hardwareMap.get(Servo.class, "outtakeArmLeft");
        slurpMotor = this.opMode.hardwareMap.get(DcMotor.class, "slurpMotor");
        claw = this.opMode.hardwareMap.get(Servo.class, "claw");
        slurp = this.opMode.hardwareMap.get(CRServo.class, "slurp");
    }



}
