/*
Copyright (c) 2018 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The present work is a derivative of a different work from PTC which was
copyrighted thusly:
===============================================================================
Copyright (c) 2018 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/

package org.firstinspires.ftc.robotcore.internal.camera.calibration;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.ClassUtil;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.robotcore.internal.system.Tracer;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

@SuppressWarnings("WeakerAccess")
public final class CameraCalibrationManager
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final String TAG = "CameraCalibrationManager";
    public static boolean Trace = true;
    protected final Tracer tracer = Tracer.create(TAG, Trace);

    protected HashMap<CameraCalibrationIdentity, List<CameraCalibration>> calibrationMap = new HashMap<>();

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public CameraCalibrationManager()
        {
        }

    public CameraCalibrationManager(@Nullable Object webcamCalibrationsParsers)
        {
        this();
        }

    public void addCalibration(CameraCalibrationIdentity vidpid, CameraCalibration calibration)
        {
        // Only one per size, but allow overriding, esp of builtin defaults by the external xml.
        // Adding a degenerate calibration will remove existing vidpid with same size.
        List<CameraCalibration> calibrations = calibrationMap.containsKey(vidpid) ? calibrationMap.get(vidpid) : new ArrayList<CameraCalibration>();
        for (int i = 0; i < calibrations.size(); i++)
            {
            CameraCalibration existing = calibrations.get(i);
            if (calibration.size.equals(existing.size))
                {
                calibrations.remove(i);
                break;
                }
            }
        if (!calibration.remove)
            {
            calibrations.add(calibration);
            calibrationMap.put(vidpid, calibrations);
            }
        }

    //----------------------------------------------------------------------------------------------
    // Parsing XML (Stubbed in EOCV-Sim)
    //----------------------------------------------------------------------------------------------

    protected void addCalibrationsFromXmlResource(int calibrationsResource)
        {
        // Stub in EOCV-Sim: no Android resources
        }

    int decodeVid(String strVid)
        {
        try {
            return Integer.decode(strVid);
            }
        catch (NumberFormatException e)
            {
            String vendorConst = "VENDOR_ID_" + strVid.toUpperCase();
            Field field = ClassUtil.getDeclaredField(UsbConstants.class, vendorConst);
            if (field != null && field.getType() == int.class)
                {
                try {
                    return field.getInt(null);
                    }
                catch (Exception ee)
                    {
                    return 0;
                    }
                }
            }
        return 0;
        }

    int decodePid(String strPid)
        {
        try {
            return Integer.decode(strPid);
            }
        catch (NumberFormatException e)
            {
            return 0;
            }
        }

    protected boolean parseBoolean(String value)
        {
        return Boolean.parseBoolean(value);
        }

    @SuppressWarnings("SameParameterValue") protected int decodeInt(String value, int defaultValue)
        {
        try {
            return Integer.decode(value);
            }
        catch (Exception e)
            {
            return defaultValue;
            }
        }

    @SuppressWarnings("SameParameterValue") protected int[] parseIntArray(int cInt, String values) throws NumberFormatException, IllegalArgumentException
        {
        if (values==null) return new int[cInt];

        values = values.replace(',', ' ');
        String[] splitValues = values.split("\\s+");
        int[] result = new int[splitValues.length];
        int i = 0;
        for (String value : splitValues)
            {
            result[i++] = Integer.decode(value);
            }
        if (result.length != cInt) throw Misc.illegalArgumentException("xml element expected to contain %d integers, contains %s", cInt, result.length);
        return result;
        }

    protected static float[] parseFloatArray(int cFloat, String values) throws NumberFormatException, IllegalArgumentException
        {
        if (values==null) return new float[cFloat];

        values = values.replace(',', ' ');
        String[] splitValues = values.split("\\s+");
        float[] result = new float[splitValues.length];
        int i = 0;
        for (String value : splitValues)
            {
            result[i++] = Float.parseFloat(value);
            }
        if (result.length != cFloat) throw Misc.illegalArgumentException("xml element expected to contain %d floats, contains %s", cFloat, result.length);
        return result;
        }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    protected List<CameraCalibration> getCalibrations(CameraCalibrationIdentity ident)
        {
        return calibrationMap.get(ident);
        }

    protected CameraCalibration[] getCameraCalibrationsWithAspectRatio(CameraCalibrationIdentity identity, Size targetSize)
        {
        double targetAspectRatio = CameraCalibration.getAspectRatio(targetSize);

        // Find all the guys with the same aspect ratio
        List<CameraCalibration> list = new ArrayList<>();
        List<CameraCalibration> calibrationList = calibrationMap.get(identity);
        if (calibrationList != null)
            {
            for (int idx = 0; idx < calibrationList.size(); idx++)
                {
                CameraCalibration calibration = calibrationList.get(idx);
                if (Misc.approximatelyEquals(calibration.getAspectRatio(), targetAspectRatio))
                    {
                    list.add(calibration);
                    }
                }
            }

        // Sort in increasing order of difference from targeted diagonal
        final double targetDiagonal = CameraCalibration.getDiagonal(targetSize);
        CameraCalibration[] result = list.toArray(new CameraCalibration[list.size()]);
        Arrays.sort(result, new Comparator<CameraCalibration>()
            {
            @Override public int compare(CameraCalibration lhs, CameraCalibration rhs)
                {
                double leftDiff = Math.abs(lhs.getDiagonal() - targetDiagonal);
                double rightDiff = Math.abs(rhs.getDiagonal() - targetDiagonal);

                double diff = leftDiff - rightDiff;
                if (diff < 0)
                    return -1;
                else if (diff > 0)
                    return 1;
                else
                    return 0;
                }
            });

        return result;
        }

    public boolean hasCalibration(CameraCalibrationIdentity identity, Size size)
        {
        return internalGetCalibration(identity, size) != null;
        }

    /**
     * Returns the calibration data, if any, for the indicated device and frame size; if
     * no such data is available, then null is returned.
     *
     * @return the calibration data, if any, for the indicated device and frame size
     * @see CameraCalibration#forUnavailable
     */
    public @NonNull CameraCalibration getCalibration(CameraCalibrationIdentity identity, Size size)
        {
        CameraCalibration result = internalGetCalibration(identity, size);
        if (result==null)
            {
            result = CameraCalibration.forUnavailable(identity, size);
            }

        tracer.trace("getCameraCalibration(): %s", result);
        return result;
        }

    protected @Nullable CameraCalibration internalGetCalibration(CameraCalibrationIdentity identity, Size size)
        {
        CameraCalibration result = null;
        if (identity != null && !identity.isDegenerate())
            {
            List<CameraCalibration> calibrationList = calibrationMap.get(identity);
            if (calibrationList != null)
                {
                // Do we have an exact match?
                for (int idx = 0; idx < calibrationList.size(); idx++)
                    {
                    CameraCalibration calibration = calibrationList.get(idx);
                    if (calibration.size.equals(size))
                        {
                        result = calibration;
                        break;
                        }
                    }

                if (result == null)
                    {
                    // If no exact match, is there one with the same aspect ratio we can scale?
                    CameraCalibration[] sameAspectRatios = getCameraCalibrationsWithAspectRatio(identity, size);
                    if (sameAspectRatios.length > 0)
                        {
                        result = sameAspectRatios[0].scaledTo(size);
                        }
                    else if (calibrationList.size() > 0)
                        {
                        result = new PlaceholderCalibratedAspectRatioMismatch(identity, size);
                        }
                    }
                }
            }
        else
            tracer.trace("getCameraCalibration(size=%s): lacking identity: no calibrations", identity, size);

        if (result != null)
            tracer.trace("%s", result);
        else
            tracer.trace("CameraCalibration(%s %dx%d)=null", identity, size.getWidth(), size.getHeight());
        return result;
        }
    }
