package org.firstinspires.ftc.teamcode.opModes;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class Sampleintake extends LinearOpMode {
    // servos that work together to go left and right
    Servo wristTurn;

    // picking up/ dropping samples and specimen
    Servo Claw;
    // servos that work together to go up and down
    Servo wristE;
    Servo wristW;


    // diff btw int and double... ?
    // should I be making these booleans or variables?
    public static double wristPositionUp = 1;
    public static double wristPositionDown = 0;
    public static double wristPositionMid = 0.5;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    //
    boolean xReverse;
    boolean yReverse;
    boolean bReverse;
    boolean openClaw;
    boolean jiggleClaw;
    boolean upPosition;
    boolean clawTurnRight;

    // Init Positions
    @Override
    public void runOpMode() throws InterruptedException {
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.scaleRange(1, 0);
        Claw.setPosition(0.5);

        wristTurn = hardwareMap.get(Servo.class, "wristN");
        wristTurn.scaleRange(0, 1);
        wristTurn.setPosition(wristPositionMid);

        wristE = hardwareMap.get(Servo.class, "wristE");
        wristE.scaleRange(0, 1);
        wristE.setPosition(wristPositionMid);

        wristW = hardwareMap.get(Servo.class, "wristW");
        wristW.scaleRange(1, 0);
        wristW.setPosition(wristPositionMid);


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
// if claw = 0 it is open , if claw = 1 it is closed
        if (openClaw) Claw.setPosition(0);
        if (!openClaw) Claw.setPosition(1);
        else Claw.setPosition(0.5);
        // (Use W & E servos?)
        if (upPosition) wristE.setPosition(0);

//jiggle Code
            if (openClaw && !jiggleClaw) wristPositionUp = 1;
            else if (openClaw && jiggleClaw) wristPositionMid = 0.5;
            else if (!openClaw && !jiggleClaw) wristPositionDown = 0;
            else wristPositionUp = 1;


        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
        }
// Opening and closing claw... (do I need to do each scenario?  ex. openClaw = openClaw..etc)

        if (gamepad1.x && !xReverse) {
            openClaw = !openClaw;
        }
// moving wrist up and down
        if (gamepad1.y && !yReverse)
            upPosition = !upPosition;

        // moving wrist right and left
        if (gamepad1.b && !bReverse)
            clawTurnRight = !clawTurnRight;

    }
}

// Specimen Sequence Code





