package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
public class GeneralHardwareMap {

    //h


        //Define runtime
        public ElapsedTime runtime = new ElapsedTime();

        //Define opMode

        public LinearOpMode opMode;


        //Define all hardware
        public VoltageSensor batteryVoltageSensor;
        public DcMotor frontLeft, frontRight, backLeft, backRight, slide;
        public DcMotor slurp;
        public DcMotor slide2;
        public Servo clawHAngle, servo, servo2, servo3, servo4, claw;
        public Servo clawVAngle, slideLAngle, slideRAngle, clawL, clawR, plane;
        public WebcamName bonoboCam;
        public HuskyLens huskyLens;
        public DistanceSensor distanceSensor;
        public ColorSensor colorSensorCenter;
        public ColorSensor colorSensorLeft;
        public ColorSensor colorSensorRight;
        public BNO055IMU gyro;//Can we do it?

        public boolean halfSpeedToggle = true;
        public boolean aLast = false;
        public static double shoulderDefaultPos = 0.35;

        public boolean drivingReverse = false;
        public boolean yLast = false;
        public static double servo1LowerScalar = .1, servo1UpperScalar=.9, servoScalarOffset = .04;
        public static double servo2LowerScalar = servo1LowerScalar + servoScalarOffset, servo2UpperScalar = servo1UpperScalar + servoScalarOffset;

//        0---------1
//        x.1----.9x
//      0---------1
//          .15-.95


        public double yMovement;
        public double xMovement;
        public double rotation;
        public double drivePower;
        public static double slidePower = 1;
        public static double pos;
        public Telemetry telemetry;
        public static double stackNum = 0;



        public GeneralHardwareMap(LinearOpMode opMode) {
            this.opMode = opMode;
        }

        public void initRANDOMOTOR(String motorName) {
            slurp = this.opMode.hardwareMap.dcMotor.get(motorName);
            slurp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public void init()
        {
            //clawHAngle = this.opMode.hardwareMap.get(Servo.class, "rotatorServo");
            servo = this.opMode.hardwareMap.get(Servo.class, "servoTest");
            servo2 = this.opMode.hardwareMap.get(Servo.class, "servoTest2");
            servo2.setDirection(Servo.Direction.REVERSE);
            servo3 = this.opMode.hardwareMap.get(Servo.class, "servoTest3");
            servo4 = this.opMode.hardwareMap.get(Servo.class, "servoTest4");
            claw = this.opMode.hardwareMap.get(Servo.class, "servoTest5");
            slide2 = this.opMode.hardwareMap.get(DcMotor.class, "slide2" );
            slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            servo.scaleRange(servo1LowerScalar, servo1UpperScalar);
            servo2.scaleRange(servo2LowerScalar, servo2UpperScalar);
        }
        public void setShoulderPosition(double position) {
            servo.setPosition(position);
            servo2.setPosition(position);
        }
        public void setSlidePosition(int pos){
            slide2.setTargetPosition(pos);
            slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide2.setPower(slidePower);
        }
        /*
        public double getColor() {
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();
            if(red > 200 && green > 200 && blue < 100){return 1;}//Yellow
            if(red > 200 && green < 100 && blue < 100){return 2;}//Red
            if(red < 100 && green < 100 && blue > 200){return 3;}//Blue
            //if(red < 100 && green < 100 && blue > 200){return 1;}MAKE A GREY
            else{return 0;}

        }
        */

/*

        public void init(String opModeType) {

            //Always intialize these
            huskyLens = this.opMode.hardwareMap.get(HuskyLens.class, "huskylens");
            distanceSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");
            colorSensorCenter = this.opMode.hardwareMap.get(ColorSensor.class, "colorSensorCenter");
            colorSensorLeft = this.opMode.hardwareMap.get(ColorSensor.class, "colorSensorRight");
            colorSensorRight = this.opMode.hardwareMap.get(ColorSensor.class, "colorSensorLeft");

            VoltageSensor batteryVoltageSensor = this.opMode.hardwareMap.voltageSensor.iterator().next();

            pos = 0.8;
            //Initialize motors only if in teleOp
            if(opModeType.equals("TELEOP")) {

                frontLeft = this.opMode.hardwareMap.dcMotor.get("frontLeft");
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                frontRight = this.opMode.hardwareMap.dcMotor.get("frontRight");
                frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                backLeft = this.opMode.hardwareMap.dcMotor.get("backLeft");
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                backRight = this.opMode.hardwareMap.dcMotor.get("backRight");
                backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }





            //telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
            //telemetry.update();
        }

        public double averageLastContents(ArrayList<Double> arr, int LOOKBACK){
            int len = arr.size();
            int count = Math.min(len, LOOKBACK);
            double sum = 0;
            for(int i = len - count; i < len; i++){
                sum += arr.get(i);
            }
            return sum/count;
        }


    /*
    public void initForApril(){
        WebcamName bonoboCam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

     */


    }


