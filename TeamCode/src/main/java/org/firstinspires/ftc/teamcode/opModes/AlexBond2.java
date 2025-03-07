package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
@Config
public class AlexBond2 extends LinearOpMode {
    public double barInterval = 0.01;
    public static double elbowDown = 0.21; //updated elbow vals
    public static double elbowHunting = 0.3;
    public static double elbowUp = 0.85;
    public static double pickUpSpecElbow = 0;
    public double wristHoriz = 0.17;
    long transferStartTime = 0;
    int transferStep = 0;
    long specStartTime = 0;
    int specStep = 0;
    long upStartTime;
    int upStep;
    // left Servo (two bar)
    Servo intakeBarL; //0C
    // right Servo
    Servo intakeBarR; //0E
    // moves entire claw up and down
    Servo intakeElbow; //3E
    // moves outtake claw up and down
    Servo shoulderR; //2C
    Servo shoulderL; //2E
    // turns claw left and right
    Servo intakeWrist; //4E
    // opens and closes claw
    Servo intakeClaw; //5E
    Servo outtakeElbow; //1C
    Servo outtakeWrist;//4C
    Servo outtakeClaw;//5C
    Servo turnHangL; //1E
    Servo turnHangR; //3C

    // moves vertical slides up and down
    DcMotor slideMotorR; //3E
    DcMotor slideMotorL;//3C
    boolean transferActive;
    boolean transferComplete;
    boolean transferMode;
    boolean transferLast;
    boolean specMode = false;
    boolean specLast;
    boolean specActive;
    boolean specComplete;
    boolean upActive;
    boolean upComplete;
    boolean outtakeClawOpen = true;
    boolean intakeClawOpen = true;
    boolean hunting;
    boolean huntingLast;
    boolean aLast;
    boolean b2Last;
    boolean yLast;
    boolean halfSpeed;
    double drivePower;
    public static int wait1 = 500;
    public static int wait2 = 500;
    public static int wait3 = 700;
    public static int wait4 = 400;
    public static int wait5 = 500;
    public static int wait6 = 500;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    //Limelight vars
    private Limelight3A limelight;
    private LLResult result;
    double[] array;
    double angle;
    double wristPos;
    long time;
    double rangifiedAngle;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.start();
        limelight.updatePythonInputs(0,0,0,0,0,0,0,0);

        intakeBarL = hardwareMap.get(Servo.class, "intakeBarL");
        intakeBarL.scaleRange(0.35, 0.67);
        intakeBarL.setPosition(1);
        intakeBarR = hardwareMap.get(Servo.class, "intakeBarR");
        intakeBarR.scaleRange(0.33, 0.65);
        intakeBarR.setPosition(0);

        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeClaw.setPosition(0.5);
        outtakeClaw.scaleRange(0.4, 0.7);
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.setPosition(0.55);
        outtakeWrist.scaleRange(0.25, 0.9);
        outtakeElbow = hardwareMap.get(Servo.class, "outtakeElbow");
        outtakeElbow.setPosition(0.75);
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        shoulderL.setPosition(0.37);
        shoulderL.scaleRange(0.1, 0.95);
        shoulderR = hardwareMap.get(Servo.class, "shoulderR");
        shoulderR.setPosition(0.63);
        shoulderR.scaleRange(0.05, 0.9);

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.scaleRange(0.2, 0.65);
        intakeClaw.setPosition(0.5);
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeWrist.setPosition(0.5);
        intakeWrist.scaleRange(0.26, 0.76);
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeElbow.setPosition(0.5);

        slideMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setPower(1);

        slideMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setTargetPosition(0);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(1);

        turnHangL = hardwareMap.get(Servo.class, "turnHangL");
        turnHangL.setPosition(0.5);
        turnHangR = hardwareMap.get(Servo.class, "turnHangR");
        turnHangR.setPosition(0.5);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        halfSpeed = false;
//pre init code above________________________________________________________________________________________________________________________________

        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
//ALWAYS
            //drive
            follower.setTeleOpMovementVectors(-gamepad2.left_stick_y * drivePower, -gamepad2.left_stick_x * drivePower, -gamepad2.right_stick_x * drivePower, true);
            follower.update();

            //half speed
            if (gamepad2.b && !b2Last) {
                halfSpeed = !halfSpeed;
            }
            drivePower = halfSpeed ? 0.25 : 1;
            b2Last = gamepad2.b;

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            //2 bar
            if (gamepad1.left_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() + barInterval * gamepad1.left_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() - barInterval * gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                intakeBarL.setPosition(intakeBarL.getPosition() - barInterval * gamepad1.right_trigger);
                intakeBarR.setPosition(intakeBarR.getPosition() + barInterval * gamepad1.right_trigger);
            }

            //manual intake wrist
            if (gamepad1.right_bumper && !gamepad1.left_bumper)
                intakeWrist.setPosition(intakeWrist.getPosition() + 0.005);
            if (gamepad1.left_bumper && !gamepad1.right_bumper)
                intakeWrist.setPosition(intakeWrist.getPosition() - 0.005);

            //outtake claw
            if (gamepad1.y && !yLast) {
                outtakeClawOpen = !outtakeClawOpen;
            }
            yLast = gamepad1.y;

            //intake claw
            if (gamepad1.a && !aLast) {
                intakeClawOpen = !intakeClawOpen;
            }
            aLast = gamepad1.a;

//BOOLEANS
            //Hi Parker
            if (intakeClawOpen) {
                intakeClaw.setPosition(1);
            } else {
                intakeClaw.setPosition(0);
            }

            if (outtakeClawOpen) {
                outtakeClaw.setPosition(0);
            } else {
                outtakeClaw.setPosition(1);
            }
//TOGGLES
    //HUNTING TOGGLE
            // Toggle hunting mode when dpad_left is pressed
            if (gamepad1.dpad_left && !huntingLast) {
                hunting = !hunting;
            }

            huntingLast = gamepad1.dpad_left;

            if (hunting) {
                intakeElbow.setPosition(elbowHunting);
                intakeClawOpen = true;

                if (gamepad1.dpad_left) intakeWrist.setPosition(0.5); //double press good vert
                else if (gamepad1.dpad_right) intakeWrist.setPosition(0); //good horizontal
                else if (gamepad1.dpad_up) intakeWrist.setPosition(0.75); //good 45 right
                else if (gamepad1.dpad_down) intakeWrist.setPosition(0.25); //good 45 left
                if (gamepad1.right_stick_button) wristAdjust();
            } else {
                if (gamepad1.dpad_right) {
                    intakeElbow.setPosition(elbowDown);
                    intakeClawOpen = false;
                }

                else if (gamepad1.dpad_up) {
                    specStep = 0;
                    specActive = true;
                    specComplete = false;
                    specStartTime = System.currentTimeMillis();
                }
                if (gamepad1.dpad_down) {
                    upStep = 0;
                    upActive = true;
                    upComplete = false;
                    upStartTime = System.currentTimeMillis();
                }
            }
            telemetry.addData("hunting?", hunting);
    //TRANSFER TOGGLE
            if (gamepad1.b && !transferLast) {
                transferMode = !transferMode;
            }

            transferLast = gamepad1.b;
            if (transferMode) {
                if (gamepad1.b && !transferActive) {
                    outtakeClawOpen = true;
                    slideMotorL.setTargetPosition(0);
                    slideMotorR.setTargetPosition(0);
                    intakeClawOpen = false;
                    intakeElbow.setPosition(elbowUp);
                    intakeWrist.setPosition(1);
                    outtakeElbow.setPosition(0.6);
                    outtakeWrist.setPosition(0.42);
                    shoulderL.setPosition(0.48);
                    shoulderR.setPosition(0.52);
                    transferStep = 0;
                    transferActive = true;
                    transferComplete = false;
                    transferStartTime = System.currentTimeMillis();
                }
            } else {
                if (gamepad1.b) {
                    outtakeClawOpen = true;
                    slideMotorL.setTargetPosition(0);
                    slideMotorR.setTargetPosition(0);
                    intakeClawOpen = true;
                    intakeElbow.setPosition(elbowHunting);
                    hunting = true;
                    intakeWrist.setPosition(1);
                    outtakeElbow.setPosition(0.6);
                    outtakeWrist.setPosition(0.42);
                    shoulderL.setPosition(0.48);
                    shoulderR.setPosition(0.52);
                }
            }
            if (gamepad1.x && !specLast) {
                specMode = !specMode;
            }

            specLast = gamepad1.x;

            if (specMode) {
                if (gamepad1.x) {
                    slideMotorR.setTargetPosition(0);
                    slideMotorL.setTargetPosition(0);
                    shoulderR.setPosition(0.9);
                    shoulderL.setPosition(0.1);
                    outtakeWrist.setPosition(0);
                    outtakeElbow.setPosition(0.5);
                    outtakeClawOpen = true;
                    slideMotorL.setTargetPosition(0);
                    slideMotorR.setTargetPosition(0);
                    //COLOR SENSOR CODE CLOSE HERE WITH SLIDES UP
                }
            }
            else {
                if (gamepad1.x) {
                    slideMotorR.setTargetPosition(0);
                    slideMotorL.setTargetPosition(0);
                    shoulderR.setPosition(0.9);
                    shoulderL.setPosition(0.1);
                    outtakeWrist.setPosition(0);
                    outtakeElbow.setPosition(0);
                    outtakeClawOpen = true;
                    slideMotorL.setTargetPosition(0);
                    slideMotorR.setTargetPosition(0);
                }
            }

//SAMPLE TRANSFER SEQUENCE
                if (transferActive && !transferComplete) {
                    long transferElapsedTime = System.currentTimeMillis() - transferStartTime;
                    switch (transferStep) {
                        //point elbow down when over sample
                        case 0:
                            intakeBarL.setPosition(0.8);
                            intakeBarR.setPosition(0.2);
                            shoulderL.setPosition(0.48);
                            shoulderR.setPosition(0.52);
                            outtakeWrist.setPosition(0);
                            outtakeElbow.setPosition(0.6);
                            if (transferElapsedTime >= wait1) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 1:
                            intakeBarL.setPosition(0.85);
                            intakeBarR.setPosition(0.15);
                            slideMotorL.setTargetPosition(0);
                            slideMotorR.setTargetPosition(0);
                            intakeWrist.setPosition(0.5);
                            if (transferElapsedTime >= wait2) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 2:
                            shoulderL.setPosition(0.44);
                            shoulderR.setPosition(0.56);
                            if (transferElapsedTime >= wait3) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 3:
                            outtakeClawOpen = false;
                            if (transferElapsedTime >= wait4) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 4:
                            intakeClawOpen = true;
                            if (transferElapsedTime >= wait4) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                        case 5:
                            intakeBarL.setPosition(0.8);
                            intakeBarR.setPosition(0.2);
                            shoulderL.setPosition(1);
                            shoulderR.setPosition(0);
                            slideMotorL.setTargetPosition(2250);
                            slideMotorR.setTargetPosition(2250);
                            outtakeWrist.setPosition(0);
                            outtakeElbow.setPosition(1);
                            transferComplete = true;
                            transferActive = false;
                            if (transferElapsedTime >= wait5) {
                                transferStep++;
                                transferStartTime = System.currentTimeMillis();
                            }
                            break;
                    }
                }
//SPEC SEQUENCE

                if (specActive && !specComplete) {
                    long specElapsedTime = System.currentTimeMillis() - specStartTime;
                    switch (specStep) {
                        case 0:
                            slideMotorR.setTargetPosition(900);
                            slideMotorL.setTargetPosition(900);
                            if (specElapsedTime >= 1000) {
                                specStep++;
                                specStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 1:
                            outtakeElbow.setPosition(1);
                            if (specElapsedTime >= 200) {
                                specStep++;
                                specStartTime = System.currentTimeMillis();
                            }
                            break;
                        case 2:
                            shoulderL.setPosition(0.25);
                            shoulderR.setPosition(0.75);
                            if (specElapsedTime >= 1000) {
                                specStep++;
                                specStartTime = System.currentTimeMillis();
                            }
                            break;
                    }
                }

            if (upActive && !upComplete) {
                long upElapsedTime = System.currentTimeMillis() - upStartTime;
                switch (upStep) {
                    case 0:
                        slideMotorL.setTargetPosition(1790);
                        slideMotorR.setTargetPosition(1790);
                        if (upElapsedTime >= 700) {
                            upStep++;
                            upStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        outtakeClawOpen = true;
                        if (upElapsedTime >= 400) {
                            upStep++;
                            upStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        shoulderL.setPosition(0.45);
                        shoulderR.setPosition(0.55);
                        if (upElapsedTime >= 500) {
                            upStep++;
                            upStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        shoulderL.setPosition(0.3);
                        shoulderR.setPosition(0.6);
                        if (upElapsedTime >= 500) {
                            upStep++;
                            upStartTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        slideMotorL.setTargetPosition(0);
                        slideMotorR.setTargetPosition(0);
                        break;
                }
            }
                telemetry.addData("specMode?", specMode);
                telemetry.addData("outtake claw pos", outtakeClaw.getPosition());
                telemetry.addData("outtake wrist pos", outtakeWrist.getPosition());
                telemetry.addData("outtake elbow pos", outtakeElbow.getPosition());
                telemetry.addData("intake claw pos", intakeClaw.getPosition());
                telemetry.addData("intake wrist pos", intakeWrist.getPosition());
                telemetry.addData("intake elbow pos", intakeElbow.getPosition());
                telemetry.addData("shoulderL pos", shoulderL.getPosition());
                telemetry.addData("shoulderR pos", shoulderR.getPosition());
                telemetry.addData("left slide pos", slideMotorL.getCurrentPosition());
                telemetry.addData("right slide pos", slideMotorR.getCurrentPosition());
                telemetry.addData("two bar L pos", intakeBarL.getPosition());
                telemetry.addData("two bar R pos", intakeBarR.getPosition());
                telemetry.addData("transfer active?", transferActive);
                telemetry.addData("transfer step", transferStep);
                telemetry.update();

            }
        limelight.stop();
        }
    public void wristAdjust(){
        intakeWrist.scaleRange(0, 1);
        result = limelight.getLatestResult();
        if (result != null) {
            telemetry.addData("result", result);
            array = result.getPythonOutput();
            angle = array[2];
            rangifiedAngle = angle/300;
            if (rangifiedAngle < 0){
                wristPos = -0.3-rangifiedAngle+0.5;
            }
            else{
                wristPos = 0.3-rangifiedAngle+0.5;
            }
            telemetry.addData("angle", angle);
            telemetry.addData("wrist pos", wristPos);
            intakeWrist.setPosition(wristPos);
        }

        intakeWrist.scaleRange(0.26, 0.76);
    }
}







