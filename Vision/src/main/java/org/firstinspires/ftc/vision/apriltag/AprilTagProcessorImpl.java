/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.vision.apriltag;

import android.graphics.Canvas;
import android.util.Log;

import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.PlaceholderCalibratedAspectRatioMismatch;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;

import java.util.ArrayList;

public class AprilTagProcessorImpl extends AprilTagProcessor
{
    public static final String TAG = "AprilTagProcessorImpl";

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();
    private boolean drawAxes;
    private boolean drawCube;
    private boolean drawOutline;
    private boolean drawTagID;

    private Mat cameraMatrix;

    private double fx;
    private double fy;
    private double cx;
    private double cy;
    private final boolean suppressCalibrationWarnings;

    private final AprilTagLibrary tagLibrary;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    private AprilTagCanvasAnnotator canvasAnnotator;

    private final DistanceUnit outputUnitsLength;
    private final AngleUnit outputUnitsAngle;

    private volatile PoseSolver poseSolver = PoseSolver.OPENCV_ITERATIVE;

    private OpenGLMatrix robotInCameraFrame;

    public AprilTagProcessorImpl(OpenGLMatrix robotInCameraFrame, double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary,
                                 boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily, int threads, boolean suppressCalibrationWarnings)
    {
        this.robotInCameraFrame = robotInCameraFrame;

        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        this.tagLibrary = tagLibrary;
        this.outputUnitsLength = outputUnitsLength;
        this.outputUnitsAngle = outputUnitsAngle;
        this.suppressCalibrationWarnings = suppressCalibrationWarnings;
        this.drawAxes = drawAxes;
        this.drawCube = drawCube;
        this.drawOutline = drawOutline;
        this.drawTagID = drawTagID;

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(tagFamily.ATLibTF.string, 3, threads);
    }

    @Override
    protected void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        // ATTEMPT 1 - If the user provided their own calibration, use that
        if (fx != 0 && fy != 0 && cx != 0 && cy != 0)
        {
            Log.d(TAG, String.format("User provided their own camera calibration fx=%7.3f fy=%7.3f cx=%7.3f cy=%7.3f",
                    fx, fy, cx, cy));
        }

        // ATTEMPT 2 - If we have valid calibration we can use, use it
        else if (calibration != null && !calibration.isDegenerate()) // needed because we may get an all zero calibration to indicate none, instead of null
        {
            fx = calibration.focalLengthX;
            fy = calibration.focalLengthY;
            cx = calibration.principalPointX;
            cy = calibration.principalPointY;

            // Note that this might have been a scaled calibration - inform the user if so
            if (calibration.resolutionScaledFrom != null)
            {
                String msg = String.format("Camera has not been calibrated for [%dx%d]; applying a scaled calibration from [%dx%d].", width, height, calibration.resolutionScaledFrom.getWidth(), calibration.resolutionScaledFrom.getHeight());

                Log.d(TAG, msg);

                if (!suppressCalibrationWarnings)
                {
                    // RobotLog.addGlobalWarningMessage(msg);
                }
            }
            // Nope, it was a full up proper calibration - no need to pester the user about anything
            else
            {
                Log.d(TAG, String.format("User did not provide a camera calibration; but we DO have a built in calibration we can use.\n [%dx%d] (NOT scaled) %s\nfx=%7.3f fy=%7.3f cx=%7.3f cy=%7.3f",
                        calibration.getSize().getWidth(), calibration.getSize().getHeight(), calibration.getIdentity().toString(), fx, fy, cx, cy));
            }
        }

        // Okay, we aren't going to have any calibration data we can use, but there are 2 cases to check
        else
        {
            // If we have a calibration on file, but with a wrong aspect ratio,
            // we can't use it, but hey at least we can let the user know about it.
            if (calibration instanceof PlaceholderCalibratedAspectRatioMismatch)
            {
                StringBuilder supportedResBuilder = new StringBuilder();

                for (CameraCalibration cal : CameraCalibrationHelper.getInstance().getCalibrations(calibration.getIdentity()))
                {
                    supportedResBuilder.append(String.format("[%dx%d],", cal.getSize().getWidth(), cal.getSize().getHeight()));
                }

                String msg = String.format("Camera has not been calibrated for [%dx%d]. Pose estimates will likely be inaccurate. However, there are built in calibrations for resolutions: %s",
                        width, height, supportedResBuilder.toString());

                Log.d(TAG, msg);

                if (!suppressCalibrationWarnings)
                {
                    // RobotLog.addGlobalWarningMessage(msg);
                }
            }

            // Nah, we got absolutely nothing
            else
            {
                String warning = "User did not provide a camera calibration, nor was a built-in calibration found for this camera. Pose estimates will likely be inaccurate.";
                Log.d(TAG, warning);

                if (!suppressCalibrationWarnings)
                {
                    // RobotLog.addGlobalWarningMessage(warning);
                }
            }

            // IN EITHER CASE, set it to *something* so we don't crash the native code
            fx = 578.272;
            fy = 578.272;
            cx = width/2;
            cy = height/2;
        }

        constructMatrix();
        canvasAnnotator = new AprilTagCanvasAnnotator(cameraMatrix);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos)
    {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = runAprilTagDetectorForMultipleTagSizes(captureTimeNanos);

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        // TODO do we need to deep copy this so the user can't mess with it before use in onDrawFrame()?
        return detections;
    }

    private Point[] nativeCornersToJavaCorners(double[][] corners)
    {
        Point[] cornerPts = new Point[4];
        for (int p = 0; p < 4; p++)
        {
            cornerPts[p] = new Point(corners[p][0], corners[p][1]);
        }

        return cornerPts;
    }

    private AprilTagPoseRaw doClusterSolve(TagClusterNotebook n, PoseSolver solver)
    {
        final int CORNERS_PER_TAG = 4;

        Point[] imagePts = new Point[n.detections.size() * CORNERS_PER_TAG];
        Point3[] idealProjectionPts = new Point3[imagePts.length];

        for (int tag = 0; tag < n.detections.size(); tag++)
        {
            PreliminaryTagDetection det = n.detections.get(tag);
            AprilTagClusterMemberMetadata metadata = n.cluster.getMemberMetadata(det.id);

            double tagsize = outputUnitsLength.fromUnit(n.cluster.distanceUnit, metadata.tagsize);
            double offsetX = outputUnitsLength.fromUnit(n.cluster.distanceUnit, metadata.positionInClusterPlane.get(0));
            double offsetY = outputUnitsLength.fromUnit(n.cluster.distanceUnit, metadata.positionInClusterPlane.get(1));
            double offsetZ = outputUnitsLength.fromUnit(n.cluster.distanceUnit, metadata.positionInClusterPlane.get(2));

            imagePts[tag * CORNERS_PER_TAG    ] = det.corners[0];
            imagePts[tag * CORNERS_PER_TAG + 1] = det.corners[1];
            imagePts[tag * CORNERS_PER_TAG + 2] = det.corners[2];
            imagePts[tag * CORNERS_PER_TAG + 3] = det.corners[3];

            idealProjectionPts[tag * CORNERS_PER_TAG    ] = new Point3(-tagsize/2 + offsetX,  tagsize/2 + offsetY, offsetZ);
            idealProjectionPts[tag * CORNERS_PER_TAG + 1] = new Point3( tagsize/2 + offsetX,  tagsize/2 + offsetY, offsetZ);
            idealProjectionPts[tag * CORNERS_PER_TAG + 2] = new Point3( tagsize/2 + offsetX, -tagsize/2 + offsetY, offsetZ);
            idealProjectionPts[tag * CORNERS_PER_TAG + 3] = new Point3(-tagsize/2 + offsetX, -tagsize/2 + offsetY, offsetZ);
        }

        Pose opencvPose = poseFromNGrt4Pts(imagePts, idealProjectionPts, cameraMatrix, solver.code);

        // Build rotation matrix
        Mat R = new Mat(3, 3, CvType.CV_32F);
        Calib3d.Rodrigues(opencvPose.rvec, R);
        float[] tmp2 = new float[9];
        R.get(0,0, tmp2);

        return new AprilTagPoseRaw(
                opencvPose.tvec.get(0,0)[0], // x
                opencvPose.tvec.get(1,0)[0], // y
                opencvPose.tvec.get(2,0)[0], // z
                new GeneralMatrixF(3,3, tmp2)); // R
    }

    private MovingStatistics solveTime = new MovingStatistics(50);

    private AprilTagPoseRaw doSingleTagPoseSolve(AprilTagMetadata metadata, PoseSolver solver, Point[] cornerPts, long ptrDetection)
    {
        AprilTagPoseRaw rawPose;
        long startSolveTime = System.currentTimeMillis();

        if (solver == PoseSolver.APRILTAG_BUILTIN)
        {
            double[] pose = ApriltagDetectionJNI.getPoseEstimate(
                    ptrDetection,
                    outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                    fx, fy, cx, cy);

            // Build rotation matrix
            float[] rotMtxVals = new float[3 * 3];
            for (int i = 0; i < 9; i++)
            {
                rotMtxVals[i] = (float) pose[3 + i];
            }

            rawPose = new AprilTagPoseRaw(
                    pose[0], pose[1], pose[2], // x y z
                    new GeneralMatrixF(3, 3, rotMtxVals)); // R
        }
        else
        {
            Pose opencvPose = poseFromTrapezoid(
                    cornerPts,
                    cameraMatrix,
                    outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                    solver.code);

            // Build rotation matrix
            Mat R = new Mat(3, 3, CvType.CV_32F);
            Calib3d.Rodrigues(opencvPose.rvec, R);
            float[] tmp2 = new float[9];
            R.get(0,0, tmp2);

            rawPose = new AprilTagPoseRaw(
                    opencvPose.tvec.get(0,0)[0], // x
                    opencvPose.tvec.get(1,0)[0], // y
                    opencvPose.tvec.get(2,0)[0], // z
                    new GeneralMatrixF(3,3, tmp2)); // R
        }

        long endSolveTime = System.currentTimeMillis();
        solveTime.add(endSolveTime-startSolveTime);

        return rawPose;
    }

    private AprilTagPoseFtc rawPoseToFtcPose(AprilTagPoseRaw rawPose)
    {
        Orientation rot = Orientation.getOrientation(rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, outputUnitsAngle);

        return new AprilTagPoseFtc(
                rawPose.x,  // x   NB: These are *intentionally* not matched directly;
                rawPose.z,  // y       this is the mapping between the AprilTag coordinate
                -rawPose.y, // z       system and the FTC coordinate system
                -rot.firstAngle, // yaw
                rot.secondAngle, // pitch
                rot.thirdAngle,  // roll
                Math.hypot(rawPose.x, rawPose.z), // range
                outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-rawPose.x, rawPose.z)), // bearing
                outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-rawPose.y, rawPose.z))); // elevation
    }

    private PreliminaryTagDetection getPreliminaryDetectionFromNative(long ptr)
    {
        int tagId = ApriltagDetectionJNI.getId(ptr);
        double[] tagCenter = ApriltagDetectionJNI.getCenterpoint(ptr);
        Point[] cornerPts = nativeCornersToJavaCorners(ApriltagDetectionJNI.getCorners(ptr));

        return new PreliminaryTagDetection(
                tagId,
                new Point(tagCenter[0], tagCenter[1]),
                cornerPts,
                ptr);
    }

    private PreliminaryFrameFindings segmentFrameFindings(long[] detectionPointers)
    {
        PreliminaryFrameFindings findings = new PreliminaryFrameFindings();

        for (long ptrDetection : detectionPointers)
        {
            PreliminaryTagDetection prelimDet = getPreliminaryDetectionFromNative(ptrDetection);

            // if we come back with a cluster match then this goes into a cluster notebook
            AprilTagClusterMetadata cluster = tagLibrary.lookupCluster(prelimDet.id);
            if (cluster != null)
            {
                findings.noteClusterTag(prelimDet, cluster);
            }
            // if no cluster match then this is just a single tag
            else
            {
                findings.noteSingleTag(prelimDet);
            }
        }

        return findings;
    }

    ArrayList<AprilTagDetection> runAprilTagDetectorForMultipleTagSizes(long captureTimeNanos)
    {
        long ptrDetectionArray = AprilTagDetectorJNI.runApriltagDetector(nativeApriltagPtr, grey.dataAddr(), grey.width(), grey.height());

        if (ptrDetectionArray == 0)
        {
            // native code gave null pointer, nothing we can do here
            // note native code will return null if there was nothing for
            // it to put into the list
            return new ArrayList<>();
        }

        long[] detectionPointers = ApriltagDetectionJNI.getDetectionPointers(ptrDetectionArray);
        ArrayList<AprilTagDetection> detections = new ArrayList<>(detectionPointers.length);

        PreliminaryFrameFindings prelimFindings = segmentFrameFindings(detectionPointers);

        // Process tag clusters
        for (TagClusterNotebook n : prelimFindings.clusterNotebooks)
        {
            AprilTagPoseRaw rawPose;
            AprilTagPoseFtc ftcPose;
            Pose3D robotPose;

            rawPose = doClusterSolve(n, poseSolver);
            ftcPose = rawPoseToFtcPose(rawPose);
            robotPose = computeRobotPose(rawPose, n.cluster.fieldPosition, n.cluster.fieldOrientation, captureTimeNanos);

            int percentClusterFound = Math.round(((float)n.detections.size() / n.cluster.clusterMembers.size()) * 100);

            detections.add(new AprilTagClusterDetection(
                    percentClusterFound, n.cluster, outputUnitsLength, ftcPose, rawPose, robotPose, captureTimeNanos)
            );
        }

        // Process single tags
        for (PreliminaryTagDetection det : prelimFindings.singleTags)
        {
            AprilTagMetadata metadata = tagLibrary.lookupTag(det.id);

            AprilTagPoseRaw rawPose;
            AprilTagPoseFtc ftcPose;
            Pose3D robotPose;

            if (metadata != null)
            {
                // note that poseSolver changes are not synchronized with anything so
                // it can change at any time, however we are effectively snapshotting
                // it when we pass it as an argument here
                rawPose = doSingleTagPoseSolve(metadata, poseSolver, det.corners, det.ptrNativeDetection);

                ftcPose = rawPoseToFtcPose(rawPose);
                robotPose = computeRobotPose(rawPose, metadata.fieldPosition, metadata.fieldOrientation, captureTimeNanos);
            }
            else
            {
                // We don't know anything about the tag size so we can't solve the pose
                rawPose = null;
                ftcPose = null;
                robotPose = null;
            }

            detections.add(new AprilTagSingleDetection(
                    det.id,
                    ApriltagDetectionJNI.getHamming(det.ptrNativeDetection),
                    ApriltagDetectionJNI.getDecisionMargin(det.ptrNativeDetection),
                    det.center, det.corners, metadata, ftcPose, rawPose, robotPose, captureTimeNanos, outputUnitsLength));
        }

        // in the null ptr case we will have returned before we get here
        ApriltagDetectionJNI.freeDetectionList(ptrDetectionArray);

        return detections;
    }

    private Pose3D computeRobotPose(AprilTagPoseRaw rawPose, VectorF fieldPosition, Quaternion fieldOrientation, long acquisitionTime)
    {
        // Compute transformation matrix of tag pose in field reference frame
        float tagInFieldX = fieldPosition.get(0);
        float tagInFieldY = fieldPosition.get(1);
        float tagInFieldZ = fieldPosition.get(2);
        OpenGLMatrix tagInFieldR = new OpenGLMatrix(fieldOrientation.toMatrix());
        OpenGLMatrix tagInFieldFrame = OpenGLMatrix.identityMatrix()
                .translated(tagInFieldX, tagInFieldY, tagInFieldZ)
                .multiplied(tagInFieldR);

        // Compute transformation matrix of camera pose in tag reference frame
        float tagInCameraX = (float) DistanceUnit.INCH.fromUnit(outputUnitsLength, rawPose.x);
        float tagInCameraY = (float) DistanceUnit.INCH.fromUnit(outputUnitsLength, rawPose.y);
        float tagInCameraZ = (float) DistanceUnit.INCH.fromUnit(outputUnitsLength, rawPose.z);
        OpenGLMatrix tagInCameraR = new OpenGLMatrix((rawPose.R));
        OpenGLMatrix cameraInTagFrame = OpenGLMatrix.identityMatrix()
                .translated(tagInCameraX, tagInCameraY, tagInCameraZ)
                .multiplied(tagInCameraR)
                .inverted();

        // Compute transformation matrix of robot pose in field frame
        OpenGLMatrix robotInFieldFrame =
                tagInFieldFrame
                .multiplied(cameraInTagFrame)
                .multiplied(robotInCameraFrame);

        // Extract robot location
        VectorF robotInFieldTranslation = robotInFieldFrame.getTranslation();
        Position robotPosition = new Position(DistanceUnit.INCH,
                robotInFieldTranslation.get(0),
                robotInFieldTranslation.get(1),
                robotInFieldTranslation.get(2),
                acquisitionTime).toUnit(outputUnitsLength);

        // Extract robot orientation
        Orientation robotInFieldOrientation = Orientation.getOrientation(robotInFieldFrame,
                AxesReference.INTRINSIC, AxesOrder.ZXY, outputUnitsAngle);
        YawPitchRollAngles robotOrientation = new YawPitchRollAngles(outputUnitsAngle,
                robotInFieldOrientation.firstAngle,
                robotInFieldOrientation.secondAngle,
                robotInFieldOrientation.thirdAngle,
                acquisitionTime);

        return new Pose3D(robotPosition, robotOrientation);
    }

    private final Object drawSync = new Object();

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        // Only one draw operation at a time thank you very much.
        // (we could be called from two different threads - viewport or camera stream)
        synchronized (drawSync)
        {
            if ((drawAxes || drawCube || drawOutline || drawTagID) && userContext != null)
            {
                canvasAnnotator.noteDrawParams(scaleBmpPxToCanvasPx, scaleCanvasDensity);

                ArrayList<AprilTagDetection> dets = (ArrayList<AprilTagDetection>) userContext;

                // For fun, draw 6DOF markers on the image.
                for(AprilTagDetection detection : dets)
                {
                    if (drawTagID)
                    {
                        canvasAnnotator.drawTagID(detection, canvas);
                    }

                    // Could be null if we couldn't solve the pose earlier due to not knowing tag size
                    if (detection.rawPose != null)
                    {
                        if (drawOutline)
                        {
                            canvasAnnotator.drawOutlineMarker(detection, canvas);
                        }
                        if (drawAxes)
                        {
                            canvasAnnotator.drawAxisMarker(detection, canvas);
                        }
                        if (drawCube)
                        {
                            canvasAnnotator.draw3dRectMarker(detection, canvas);
                        }
                    }
                }
            }
        }
    }

    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    @Override
    public void setPoseSolver(PoseSolver poseSolver)
    {
        this.poseSolver = poseSolver;
    }

    @Override
    public int getPerTagAvgPoseSolveTime()
    {
        return (int) Math.round(solveTime.getMean());
    }

    public ArrayList<AprilTagDetection> getDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getFreshDetections()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Converts an AprilTag pose to an OpenCV pose
     * @param aprilTagPose pose to convert
     * @return OpenCV output pose
     */
    static Pose aprilTagPoseToOpenCvPose(AprilTagPoseRaw aprilTagPose)
    {
        Pose pose = new Pose();
        pose.tvec.put(0,0, aprilTagPose.x);
        pose.tvec.put(1,0, aprilTagPose.y);
        pose.tvec.put(2,0, aprilTagPose.z);

        Mat R = new Mat(3, 3, CvType.CV_32F);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.put(i,j, aprilTagPose.R.get(i,j));
            }
        }

        Calib3d.Rodrigues(R, pose.rvec);

        return pose;
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsize the original length of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    static Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsize, int solveMethod)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsize/2, tagsize/2, 0);
        arrayPoints3d[1] = new Point3(tagsize/2, tagsize/2, 0);
        arrayPoints3d[2] = new Point3(tagsize/2, -tagsize/2, 0);
        arrayPoints3d[3] = new Point3(-tagsize/2, -tagsize/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false, solveMethod);

        return pose;
    }

    static Pose poseFromNGrt4Pts(Point[] imagePts, Point3[] idealSpacialProjectedPts, Mat cameraMatrix, int solveMethod)
    {
        MatOfPoint2f points2d = new MatOfPoint2f(imagePts);
        MatOfPoint3f points3d = new MatOfPoint3f(idealSpacialProjectedPts);

        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false, solveMethod);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    static class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }

    static class TagClusterNotebook
    {
        private AprilTagClusterMetadata cluster;
        private ArrayList<PreliminaryTagDetection> detections = new ArrayList<>();

        TagClusterNotebook(AprilTagClusterMetadata cluster)
        {
            this.cluster = cluster;
        }

        boolean isFor(AprilTagClusterMetadata cluster)
        {
            return this.cluster == cluster;
        }

        void note(PreliminaryTagDetection det)
        {
            detections.add(det);
        }
    }

    static class PreliminaryTagDetection
    {
        final int id;
        final Point center;
        final Point[] corners;
        final long ptrNativeDetection;

        PreliminaryTagDetection(int id, Point center, Point[] corners, long ptrNativeDetection)
        {
            this.id = id;
            this.center = center;
            this.corners = corners;
            this.ptrNativeDetection = ptrNativeDetection;
        }
    }

    static class PreliminaryFrameFindings
    {
        ArrayList<TagClusterNotebook> clusterNotebooks;
        ArrayList<PreliminaryTagDetection> singleTags;

        PreliminaryFrameFindings()
        {
            clusterNotebooks = new ArrayList<>();
            singleTags = new ArrayList<>();
        }

        void noteSingleTag(PreliminaryTagDetection detection)
        {
            singleTags.add(detection);
        }

        void noteClusterTag(PreliminaryTagDetection detection, AprilTagClusterMetadata parentCluster)
        {
            // do we already have a notebook that matches?
            for (TagClusterNotebook notebook : clusterNotebooks)
            {
                if (notebook.isFor(parentCluster))
                {
                    notebook.note(detection);
                    return;
                }
            }

            // no, we need to make a new one
            TagClusterNotebook n = new TagClusterNotebook(parentCluster);
            n.note(detection);
            clusterNotebooks.add(n);
        }
    }
}