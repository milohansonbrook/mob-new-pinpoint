package org.firstinspires.ftc.teamcode.AutonPoses;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class RedClipPoses {
    public static double xPosStart = 131.5033377837116, yPosStart = 99.01201602136182, headingStart = 0;
    public static double dBPlacex, dBPlacey;
    public static double fBMidx, fBMidy;
    public static double fBPickx, fBPicky;
    public static double fBDropx, fBDropy;
    public static double sBPickx, sBPicky;
    public static double sBDropx, sBDropy;
    public static double tBPickx, tBPicky;
    public static double tBDropx, tBDropy;
    public static double parkx, parky;
    public Pose start, dBPlace, fBMid, fBPick, fBDrop, sBPick, sBDrop, tBPick, tBDrop, park;
    public void init(){
        start = new Pose(xPosStart, yPosStart, Math.toRadians(headingStart));
        dBPlace = new Pose(dBPlacex, dBPlacey, Math.toRadians(headingStart));
        fBMid = new Pose(fBMidx, fBMidy, Math.toRadians(headingStart));
        fBPick = new Pose(fBPickx, fBPicky, Math.toRadians(headingStart));
        fBDrop = new Pose(fBDropx, fBDropy, Math.toRadians(headingStart));
        sBPick = new Pose(sBPickx, sBPicky, Math.toRadians(headingStart));
        sBDrop = new Pose(sBDropx, sBDropy, Math.toRadians(headingStart));
        tBPick = new Pose(tBPickx, tBPicky, Math.toRadians(headingStart));
        tBDrop = new Pose(tBDropx, tBDropy, Math.toRadians(headingStart));
        park = new Pose(parkx, parky, Math.toRadians(headingStart));
    }
}
