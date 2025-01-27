package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .001989436789;
        TwoWheelConstants.strafeTicksToInches = .001989436789;
        TwoWheelConstants.forwardY = 7.6/.001989436789;
        TwoWheelConstants.strafeX = -1/.001989436789;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "leftBack";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "rightBack";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    }
}




