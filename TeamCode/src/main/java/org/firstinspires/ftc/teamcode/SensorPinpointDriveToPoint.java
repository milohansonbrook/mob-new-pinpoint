package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="Pinpoint Navigation Example", group="Pinpoint")
//@Disabled

public class SensorPinpointDriveToPoint extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine{
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3;
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 2000, 0, AngleUnit.DEGREES, 90);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,500,0, AngleUnit.DEGREES,0);


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        nav.initializeMotors();

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            if(stateMachine == StateMachine.WAITING_FOR_START){
                stateMachine = StateMachine.DRIVE_TO_TARGET_1;
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_1) {
                if (nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 3)) {
                    telemetry.addLine("at position #1!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                }
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_2){
                if (nav.driveTo(odo.getPosition(), TARGET_2, 0.7, 3)) {
                    telemetry.addLine("at position #2!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                }
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_3){
                if (nav.driveTo(odo.getPosition(), TARGET_3, 0.7, 3)){
                    telemetry.addLine("at position #3!");
                    stateMachine = StateMachine.AT_TARGET;
                }
            }

            telemetry.addData("current state:",stateMachine);
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();

        }
    }}
