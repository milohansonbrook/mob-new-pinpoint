package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
//        TwoWheelConstants.forwardTicksToInches = .001989436789;
//        TwoWheelConstants.strafeTicksToInches = .001989436789;
//        TwoWheelConstants.forwardY = 7.6/.001989436789;
//        TwoWheelConstants.strafeX = -1/.001989436789;
//        TwoWheelConstants.forwardEncoder_HardwareMapName = "leftBack";
//        TwoWheelConstants.strafeEncoder_HardwareMapName = "rightBack";
//        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
//        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
//        TwoWheelConstants.IMU_HardwareMapName = "imu";
//        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        PinpointConstants.forwardY = 7.2;
        PinpointConstants.strafeX = -1;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}




