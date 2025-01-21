package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class MotorAndServoPoses extends LinearOpMode {
    Servo intakeRight;
    Servo claw;
    Servo outtakeWrist;
    Servo outtakeArmRight;
    Servo outtakeArmLeft;
    DcMotor grabMotorL;
    DcMotor grabMotorR;
    Servo twoBarLeft;
    Servo twoBarRight;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        //intakeRight.scaleRange(slurpLowerBound, slurpUpperBound);
        intakeRight.setPosition(1);

        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.scaleRange(0.3, 0.58);
        outtakeWrist.setPosition(0);

        outtakeArmRight = hardwareMap.get(Servo.class, "outtakeArmRight");
        outtakeArmRight.scaleRange(0.25, 0.8);
        outtakeArmRight.setPosition(0.3);

        outtakeArmLeft = hardwareMap.get(Servo.class, "outtakeArmLeft");
        outtakeArmLeft.scaleRange(0.2, 0.75);
        outtakeArmLeft.setPosition(0.7);

        grabMotorL = hardwareMap.get(DcMotor.class, "grabMotorL");
        grabMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorL.setTargetPosition(0);
        grabMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorL.setPower(1);

        grabMotorR = hardwareMap.get(DcMotor.class, "grabMotorR");
        grabMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        grabMotorR.setTargetPosition(0);
        grabMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotorR.setPower(1);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0.55, 0.65);
        claw.setPosition(0);

        grabMotorL = hardwareMap.get(DcMotor.class, "grabMotorL");
        grabMotorR = hardwareMap.get(DcMotor.class, "grabMotorR");

        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("inTakeRight", intakeRight.getPosition());
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("outtakeWrist", outtakeWrist.getPosition());
            telemetry.addData("outtakeArmRight", outtakeArmRight.getPosition());
            telemetry.addData("outtakeArmLeft", outtakeArmLeft.getPosition());
            telemetry.addData("twoBarLeft", twoBarLeft.getPosition());
            telemetry.addData("twoBarRight", twoBarRight.getPosition());
            telemetry.addData("grabMotorL", grabMotorL.getCurrentPosition());
            telemetry.addData("grabMotorR", grabMotorR.getCurrentPosition());

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            telemetry.update();

        }
    }
}
