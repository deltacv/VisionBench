package org.firstinspires.ftc.robotcore.internal.camera.calibration;

import org.firstinspires.ftc.robotcore.external.android.util.Size;

public class PlaceholderCalibratedAspectRatioMismatch extends CameraCalibration
{
    public PlaceholderCalibratedAspectRatioMismatch(CameraCalibrationIdentity ident, Size size)
    {
        super(ident, size, 0, 0, 0, 0, new float[8], false, true);
    }
}
