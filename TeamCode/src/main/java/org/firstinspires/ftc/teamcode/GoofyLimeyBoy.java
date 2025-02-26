package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


@TeleOp
public class GoofyLimeyBoy extends LinearOpMode {
    private Limelight3A limelight;
    private LLResult result;
    double[] array;
    double angle;
    double wristPos;
    long time;
    double rangifiedAngle;

    Servo intakeWrist;

    public void runOpMode() throws InterruptedException{

        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        //bounds are .2 and .8
        intakeWrist.setPosition(0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

//        limelight.pipelineSwitch(0);
        /*
         * Starts polling for data.
         */

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            time = System.currentTimeMillis();
            telemetry.update();
            result = limelight.getLatestResult();

            if (result != null) {
                if (time%300==0) {
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
                }
            }

    }
}
