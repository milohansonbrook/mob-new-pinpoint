//package org.firstinspires.ftc.teamcode.Auton;
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@Disabled
//@Config
//@Autonomous (name="Auton Bucket", group = "Autonomous")
//public class TestClip extends OpMode{
//
//    public static double specPlace1x = 27.414178322619343;
//    public static double specPlace1y = 12.264530452217643;
//
//    public static double specPickupx = 0.962159689955824;
//    public static double specPickupy = -30.779298797367126;
//
//    public static double specPlace2x = 27.414178322619343;
//    public static double specPlace2y = 16.44391157495694;
//
//    public static double specPlace3x = 28.302859809454972;
//    public static double specPlace3y = 20.027724090335876;
//
//
//
//    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//
//  /*
//
//    Clip pos3: x: 28.302859809454972
//    y: 20.027724090335876
//
//    Interim: x: 30.110125203770917
//    y: -19.590336356576035
//
//    Infront+ turn moment: x: 48.02768497016486
//    y: -18.983579620601624
//
//    Prepush1: x: 47.447793314776085
//    y: -28.27796755813238
//
//    Lepush1: x: 8.698523573988066 y: -28.891685666061765
//
//    Interim2: x: 48.60065129798228 y: -27.91693740003691
//
//    Prepush2: x: 49.29028128075788 y: -35.274533249261815
//
//    Lepush2: x: 8.806367889164001 y: -41.00909856360728
//
//    Interim3: : x: 48.60065129798228 y: -27.91693740003691
//
//    Prepush3: x: 49.764264925258374 y: -43.02251860851378
//
//    Lepush3: x: 8.579261359267347 y: -48.7841796875
//
//*/
//    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//    private final Pose clipPlace = new Pose(specPlace1x, specPlace1y, Math.toRadians(0));
//
//
//    @Override
//    public void init() {
//
//    }
//
//    @Override
//    public void loop() {
//
//    }
//}
