package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class DriveToPoint {
    private static final double X_Y_TOLERANCE = 12;
    private static final double YAW_TOLERANCE = 1;

    private static final double FORWARD_P = 0.03;
    private static final double FORWARD_D = 0.002;
    private static final double FORWARD_ACCEL = 2.0;

    private static final double PERPENDICULAR_P = 0.03;
    private static final double PERPENDICULAR_D = 0.0005;
    private static final double PERPENDICULAR_ACCEL = 2;
    /*todo: make the forward/perpendicular pid values smarter. Instead of just using perpendicular for strafe. Use it for the smaller error axis, maybe a cutoff?
       consider if this is even better than a well tuned single PID loop*/

    private static final double YAW_P = 5.0;
    private static final double YAW_D = 0.0;
    private static final double YAW_ACCEL = 2.0;

    private double X_POSITION = 0;
    private double Y_POSITION = 0;
    private double HEADING    = 0;

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private ElapsedTime holdTimer = new ElapsedTime();
    private ElapsedTime currentTime = new ElapsedTime();

    private PIDLoops xPID = new PIDLoops();
    private PIDLoops yPID = new PIDLoops();
    private PIDLoops hPID = new PIDLoops();

    private LinearOpMode myOpMode;

    private enum Direction {
        x,
        y,
        h;
    }

    public void setCoefficients(double pGain, double dGain, double accel){
        //todo: implement this
    }


    public DriveToPoint(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initializeMotors() {
        leftFrontDrive = setupDriveMotor("leftFrontDrive", DcMotorSimple.Direction.REVERSE);
        rightFrontDrive = setupDriveMotor("rightFrontDrive", DcMotorSimple.Direction.FORWARD);
        leftBackDrive = setupDriveMotor("leftBackDrive", DcMotorSimple.Direction.REVERSE);
        rightBackDrive = setupDriveMotor("rightBackDrive", DcMotorSimple.Direction.FORWARD);
    }

    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
        boolean atTarget;
        double xPWR    = calculatePID(currentPosition,targetPosition,Direction.x);
        double yPWR    = calculatePID(currentPosition,targetPosition,Direction.y);
        double hOutput = calculatePID(currentPosition,targetPosition,Direction.h);

        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        double cosine  = Math.cos(heading);
        double sine    = Math.sin(heading);

        double xOutput = (xPWR * cosine) + (yPWR * sine);
        double yOutput = (xPWR * sine) - (yPWR * cosine);

        driveMecanums(xOutput*power, yOutput*power, hOutput*power);

        if(inBounds(currentPosition,targetPosition)){
            atTarget = true;
        }
        else {
            holdTimer.reset();
            atTarget = false;
        }

        if(atTarget && holdTimer.time() > holdTime){
            return true;
        }
        return false;
    }

    private void driveMecanums(double forward, double strafe, double yaw) {
        double leftFront = forward - -strafe - yaw;
        double rightFront = forward + -strafe + yaw;
        double leftBack = forward + -strafe - yaw;
        double rightBack = forward - -strafe + yaw;

        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return aMotor;
    }



    private double calculatePID(Pose2D currentPosition, Pose2D targetPosition, Direction direction){
        if(direction == Direction.x){
            double xError = targetPosition.getX(MM) - currentPosition.getX(MM);
            return xPID.calculateAxisPID(xError, FORWARD_P, FORWARD_D, currentTime.time());
        }
        if(direction == Direction.y){
            double yError = targetPosition.getY(MM) - currentPosition.getY(MM);
            return yPID.calculateAxisPID(yError, PERPENDICULAR_P, PERPENDICULAR_D, currentTime.time());
        }
        if(direction == Direction.h){
            double hError = targetPosition.getHeading(AngleUnit.RADIANS) - currentPosition.getHeading(AngleUnit.RADIANS);
            return hPID.calculateAxisPID(hError, YAW_P, YAW_D, currentTime.time());
        }
        return 0;
    }

    private boolean inBounds (Pose2D currPose, Pose2D trgtPose){
        boolean xOutOfBounds = currPose.getX(MM) > (trgtPose.getX(MM) - X_Y_TOLERANCE) && currPose.getX(MM) < (trgtPose.getX(MM) + X_Y_TOLERANCE);
        boolean yOutOfBounds = currPose.getY(MM) > (trgtPose.getY(MM) - X_Y_TOLERANCE) && currPose.getY(MM) < (trgtPose.getY(MM) + X_Y_TOLERANCE);
        boolean hOutOfBounds = currPose.getHeading(DEGREES) > (trgtPose.getHeading(DEGREES) - YAW_TOLERANCE) &&
                currPose.getHeading(DEGREES) < (trgtPose.getHeading(DEGREES) + YAW_TOLERANCE);

        return xOutOfBounds && yOutOfBounds && hOutOfBounds;
    }
}

class PIDLoops{
    private double previousError;
    private double previousTime;

    double calculateAxisPID(double error, double pGain, double dGain, double currentTime){
        double p = error * pGain;
        double d = dGain * (previousError - error) / (currentTime - previousTime);
        double output = p+d;

        previousError = error;
        previousTime  = currentTime;

        double max = Math.abs(output);
        if(max > 1.0){
            output /= max;
        }

        return output;
    }
}
