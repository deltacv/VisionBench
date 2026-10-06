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
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

public class AprilTagCanvasAnnotator
{
    final Mat cameraMatrix;
    float bmpPxToCanvasPx;
    float canvasDensityScale;

    LinePaint redAxisPaint = new LinePaint(Color.RED);
    LinePaint greenAxisPaint = new LinePaint(Color.GREEN);
    LinePaint blueAxisPaint = new LinePaint(Color.BLUE);

    LinePaint boxPillarPaint = new LinePaint(Color.rgb(7,197,235));
    LinePaint boxTopPaint = new LinePaint(Color.GREEN);

    static class LinePaint extends Paint
    {
        public LinePaint(int color)
        {
            setColor(color);
            setAntiAlias(true);
            setStrokeCap(Paint.Cap.ROUND);
        }
    }

    Paint textPaint;
    Paint rectPaint;

    public AprilTagCanvasAnnotator(Mat cameraMatrix)
    {
        this.cameraMatrix = cameraMatrix;

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setAntiAlias(true);
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        rectPaint = new Paint();
        rectPaint.setAntiAlias(true);
        rectPaint.setColor(Color.rgb(12, 145, 201));
        rectPaint.setStyle(Paint.Style.FILL);
    }

    public void noteDrawParams(float bmpPxToCanvasPx, float canvasDensityScale)
    {
        if (bmpPxToCanvasPx != this.bmpPxToCanvasPx || canvasDensityScale != this.canvasDensityScale)
        {
            this.bmpPxToCanvasPx = bmpPxToCanvasPx;
            this.canvasDensityScale = canvasDensityScale;

            textPaint.setTextSize(40*canvasDensityScale);
        }
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     */
    void drawAxisMarker(AprilTagDetection detection, Canvas canvas)
    {
        float tagsizeToUseForAxisMarker;

        if (detection instanceof AprilTagClusterDetection)
        {
            AprilTagClusterMetadata metadata = ((AprilTagClusterDetection) detection).metadata;

            tagsizeToUseForAxisMarker = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.largestTagSize);
        }
        else if (detection instanceof AprilTagSingleDetection)
        {
            AprilTagMetadata metadata = ((AprilTagSingleDetection) detection).metadata;

            tagsizeToUseForAxisMarker = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.tagsize);
        }
        else
        {
            return;
        }

        AprilTagProcessorImpl.Pose pose = AprilTagProcessorImpl.aprilTagPoseToOpenCvPose(detection.rawPose);

        // in meters, actually.... will be mapped to screen coords
        float axisLength = (float) (tagsizeToUseForAxisMarker / 2.0);

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(-axisLength,0,0),
                new Point3(0,-axisLength,0),
                new Point3(0,0,-axisLength)
        );

        // Project those points onto the image
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, pose.rvec, pose.tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // The projection we did was good for the original resolution image, but now
        // we need to scale those to their locations on the canvas.
        for (Point p : projectedPoints)
        {
            p.x *= bmpPxToCanvasPx;
            p.y *= bmpPxToCanvasPx;
        }

        // Use the 3D distance to the target, as well as the physical size of the
        // target in the real world to scale the thickness of lines.
        double dist3d = Math.sqrt(Math.pow(detection.rawPose.x, 2) + Math.pow(detection.rawPose.y, 2) + Math.pow(detection.rawPose.z, 2));
        float axisThickness = (float) ((5 / dist3d) * (tagsizeToUseForAxisMarker / 0.166) * bmpPxToCanvasPx); // looks about right I guess

        redAxisPaint.setStrokeWidth(axisThickness);
        greenAxisPaint.setStrokeWidth(axisThickness);
        blueAxisPaint.setStrokeWidth(axisThickness);

        // Now draw the axes
        canvas.drawLine((float)projectedPoints[0].x,(float)projectedPoints[0].y, (float)projectedPoints[1].x, (float)projectedPoints[1].y, redAxisPaint);
        canvas.drawLine((float)projectedPoints[0].x,(float)projectedPoints[0].y, (float)projectedPoints[2].x, (float)projectedPoints[2].y, greenAxisPaint);
        canvas.drawLine((float)projectedPoints[0].x,(float)projectedPoints[0].y, (float)projectedPoints[3].x, (float)projectedPoints[3].y, blueAxisPaint);
    }

    /**
     * Draw a 3D cube marker on a detection
     *
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     */
    void draw3dRectMarker(AprilTagDetection detection, Canvas canvas)
    {
        AprilTagProcessorImpl.Pose pose = AprilTagProcessorImpl.aprilTagPoseToOpenCvPose(detection.rawPose);

        float xMin, xMax;
        float yMin, yMax;
        float zBottom, zTop;
        float sizeForPainting;

        if (detection instanceof AprilTagClusterDetection)
        {
            AprilTagClusterMetadata metadata = ((AprilTagClusterDetection) detection).metadata;

            xMin = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.xExtentMin);
            xMax = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.xExtentMax);
            yMin = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.yExtentMin);
            yMax = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.yExtentMax);

            // Anchor the box to the tags' actual Z location in the cluster plane (which may be
            // displaced from the cluster origin) rather than to the z=0 origin plane. The box
            // rises from the farthest tag plane toward the camera by the visual depth (zExtent).
            float zDepth = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.largestTagSize);
            zBottom = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.zExtentMax);
            zTop = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.zExtentMin) - zDepth;
            sizeForPainting = (float) DistanceUnit.INCH.fromUnit(metadata.distanceUnit, metadata.largestTagSize);
        }
        else if (detection instanceof AprilTagSingleDetection)
        {
            AprilTagMetadata metadata = ((AprilTagSingleDetection) detection).metadata;

            float tagSize = (float) detection.distanceUnit.fromUnit(metadata.distanceUnit, metadata.tagsize);
            xMin = -tagSize/2;
            xMax =  tagSize/2;
            yMin = -tagSize/2;
            yMax =  tagSize/2;
            zBottom = 0;
            zTop = -tagSize;
            sizeForPainting = (float) DistanceUnit.INCH.fromUnit(metadata.distanceUnit, metadata.tagsize);
        }
        else
        {
            return;
        }

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                // bottom plane
                new Point3(xMin, yMax, zBottom),
                new Point3(xMax, yMax, zBottom),
                new Point3(xMax, yMin, zBottom),
                new Point3(xMin, yMin, zBottom),
                // top plane
                new Point3(xMin, yMax, zTop),
                new Point3(xMax, yMax, zTop),
                new Point3(xMax, yMin, zTop),
                new Point3(xMin, yMin, zTop));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, pose.rvec, pose.tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // The projection we did was good for the original resolution image, but now
        // we need to scale those to their locations on the canvas.
        for (Point p : projectedPoints)
        {
            p.x *= bmpPxToCanvasPx;
            p.y *= bmpPxToCanvasPx;
        }

        // Use the 3D distance to the target, as well as the physical size of the
        // target in the real world to scale the thickness of lines.
        double dist3d = Math.sqrt(Math.pow(detection.rawPose.x, 2) + Math.pow(detection.rawPose.y, 2) + Math.pow(detection.rawPose.z, 2));
        float thickness = (float) ((3.5 / dist3d) * (sizeForPainting / 0.166) * bmpPxToCanvasPx); // looks about right I guess

        boxPillarPaint.setStrokeWidth(thickness);
        boxTopPaint.setStrokeWidth(thickness);

        float[] pillarPts = new float[16];

        // Pillars
        for(int i = 0; i < 4; i++)
        {
            pillarPts[i*4+0] = (float) projectedPoints[i].x;
            pillarPts[i*4+1] = (float) projectedPoints[i].y;

            pillarPts[i*4+2] = (float) projectedPoints[i+4].x;
            pillarPts[i*4+3] = (float) projectedPoints[i+4].y;
        }

        canvas.drawLines(pillarPts, boxPillarPaint);

        // Top lines
        float[] topPts = new float[] {
                (float) projectedPoints[4].x, (float) projectedPoints[4].y, (float) projectedPoints[5].x, (float) projectedPoints[5].y,
                (float) projectedPoints[5].x, (float) projectedPoints[5].y, (float) projectedPoints[6].x, (float) projectedPoints[6].y,
                (float) projectedPoints[6].x, (float) projectedPoints[6].y, (float) projectedPoints[7].x, (float) projectedPoints[7].y,
                (float) projectedPoints[4].x, (float) projectedPoints[4].y, (float) projectedPoints[7].x, (float) projectedPoints[7].y
        };

        canvas.drawLines(topPts, boxTopPaint);
    }

    void drawOutlineMarker(AprilTagDetection detection, Canvas canvas)
    {
        if (detection instanceof AprilTagSingleDetection)
        {
            drawOutlineMarkerForSingleTag((AprilTagSingleDetection) detection, canvas);
        }
        else if (detection instanceof AprilTagClusterDetection)
        {
            drawOutlineMarkerForCluster((AprilTagClusterDetection) detection, canvas);
        }
    }

    private Point[] projectCornersAndCenterForCluster(AprilTagClusterDetection detection)
    {
        AprilTagProcessorImpl.Pose pose = AprilTagProcessorImpl.aprilTagPoseToOpenCvPose(detection.rawPose);

        double xMin = detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, detection.metadata.xExtentMin);
        double xMax = detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, detection.metadata.xExtentMax);
        double yMin = detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, detection.metadata.yExtentMin);
        double yMax = detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, detection.metadata.yExtentMax);

        // The outline lies on the tags' plane, which may be displaced from the cluster origin in Z.
        // Use the midpoint of the cluster's Z extent (equal to the displacement for a coplanar cluster)
        // so the outline and ID label track the tags rather than the z=0 origin plane.
        double zMin = detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, detection.metadata.zExtentMin);
        double zMax = detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, detection.metadata.zExtentMax);
        double zPlane = (zMin + zMax) / 2;

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(xMin, yMax, zPlane),
                new Point3(xMax, yMax, zPlane),
                new Point3(xMax, yMin, zPlane),
                new Point3(xMin, yMin, zPlane),
                new Point3((xMin + xMax) / 2, (yMin + yMax) / 2, zPlane), // center of bounding rect
                new Point3(0, 0, 0)); // origin (where the axis marker is drawn)

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, pose.rvec, pose.tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);

        return matProjectedPoints.toArray();
    }

    private void drawOutlineMarkerForCluster(AprilTagClusterDetection detection, Canvas canvas)
    {
        float xSizePreScale = (float) (detection.metadata.xExtentMax - detection.metadata.xExtentMin);
        float xSize = (float) detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, xSizePreScale);

        Point[] projectedPoints = projectCornersAndCenterForCluster(detection);

        // The projection we did was good for the original resolution image, but now
        // we need to scale those to their locations on the canvas.
        for (Point p : projectedPoints)
        {
            p.x *= bmpPxToCanvasPx;
            p.y *= bmpPxToCanvasPx;
        }

        // Use the 3D distance to the target, as well as the physical size of the
        // target in the real world to scale the thickness of lines.
        double dist3d = Math.sqrt(Math.pow(detection.rawPose.x, 2) + Math.pow(detection.rawPose.y, 2) + Math.pow(detection.rawPose.z, 2));
        float thickness = (float) ((3.5 / dist3d) * (xSize / 0.166) * bmpPxToCanvasPx); // looks about right I guess

        redAxisPaint.setStrokeWidth(thickness);
        greenAxisPaint.setStrokeWidth(thickness);
        blueAxisPaint.setStrokeWidth(thickness);

        canvas.drawLine(
                (float)projectedPoints[0].x,(float)projectedPoints[0].y,
                (float)projectedPoints[1].x, (float)projectedPoints[1].y,
                redAxisPaint);

        canvas.drawLine(
                (float)projectedPoints[1].x,(float)projectedPoints[1].y,
                (float)projectedPoints[2].x, (float)projectedPoints[2].y,
                greenAxisPaint);

        canvas.drawLine(
                (float)projectedPoints[0].x,(float)projectedPoints[0].y,
                (float)projectedPoints[3].x, (float)projectedPoints[3].y,
                blueAxisPaint);

        canvas.drawLine(
                (float)projectedPoints[2].x,(float)projectedPoints[2].y,
                (float)projectedPoints[3].x, (float)projectedPoints[3].y,
                blueAxisPaint);
    }

    /**
     * Draw an outline marker on the detection
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     */
    private void drawOutlineMarkerForSingleTag(AprilTagSingleDetection detection, Canvas canvas)
    {
        double tagsize = detection.distanceUnit.fromUnit(detection.metadata.distanceUnit, detection.metadata.tagsize);

        // Use the 3D distance to the target, as well as the physical size of the
        // target in the real world to scale the thickness of lines.
        double dist3d = Math.sqrt(Math.pow(detection.rawPose.x, 2) + Math.pow(detection.rawPose.y, 2) + Math.pow(detection.rawPose.z, 2));
        float axisThickness = (float) ((5 / dist3d) * (tagsize / 0.166) * bmpPxToCanvasPx); // looks about right I guess

        redAxisPaint.setStrokeWidth(axisThickness);
        greenAxisPaint.setStrokeWidth(axisThickness);
        blueAxisPaint.setStrokeWidth(axisThickness);

        canvas.drawLine(
                (float)detection.corners[0].x*bmpPxToCanvasPx,(float)detection.corners[0].y*bmpPxToCanvasPx,
                (float)detection.corners[1].x*bmpPxToCanvasPx, (float)detection.corners[1].y*bmpPxToCanvasPx,
                redAxisPaint);

        canvas.drawLine(
                (float)detection.corners[1].x*bmpPxToCanvasPx,(float)detection.corners[1].y*bmpPxToCanvasPx,
                (float)detection.corners[2].x*bmpPxToCanvasPx, (float)detection.corners[2].y*bmpPxToCanvasPx,
                greenAxisPaint);

        canvas.drawLine(
                (float)detection.corners[0].x*bmpPxToCanvasPx,(float)detection.corners[0].y*bmpPxToCanvasPx,
                (float)detection.corners[3].x*bmpPxToCanvasPx, (float)detection.corners[3].y*bmpPxToCanvasPx,
                blueAxisPaint);

        canvas.drawLine(
                (float)detection.corners[2].x*bmpPxToCanvasPx,(float)detection.corners[2].y*bmpPxToCanvasPx,
                (float)detection.corners[3].x*bmpPxToCanvasPx, (float)detection.corners[3].y*bmpPxToCanvasPx,
                blueAxisPaint);
    }

    /**
     * Draw the Tag's ID on the tag
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     */
    void drawTagID(AprilTagDetection detection, Canvas canvas)
    {
        if (detection instanceof AprilTagClusterDetection)
        {
            String text = ((AprilTagClusterDetection) detection).metadata.shortName;
            Point[] projectedCorners = projectCornersAndCenterForCluster((AprilTagClusterDetection) detection);

            Point lowerLeftCorner = projectedCorners[0];
            Point lowerRightCorner = projectedCorners[1];
            Point rectCenter = projectedCorners[4];
            Point origin = projectedCorners[5];

            // Always draw the name at the origin (where the axis marker is drawn).
            drawLabel(text, origin, lowerLeftCorner, lowerRightCorner, canvas);

            // Also draw it at the center of the bounding rectangle, but only if it's far enough
            // from the origin to avoid the two labels overlapping.
            double centerSpacing = Math.hypot(rectCenter.x - origin.x, rectCenter.y - origin.y) * bmpPxToCanvasPx;
            if (centerSpacing > 50 * canvasDensityScale)
            {
                drawLabel(text, rectCenter, lowerLeftCorner, lowerRightCorner, canvas);
            }
        }
        else if (detection instanceof AprilTagSingleDetection)
        {
            AprilTagSingleDetection asSingle = (AprilTagSingleDetection) detection;

            drawLabel(String.format("ID %d", asSingle.id), asSingle.center, asSingle.corners[0], asSingle.corners[1], canvas);
        }
    }

    /**
     * Draw a rounded-rectangle text label, centered on and rotated to align with the detection.
     *
     * @param text the text to draw
     * @param center the point (in bitmap px) the label is centered on
     * @param lowerLeftCorner a corner used (with lowerRightCorner) to compute the label rotation
     * @param lowerRightCorner a corner used (with lowerLeftCorner) to compute the label rotation
     * @param canvas the canvas to draw on
     */
    private void drawLabel(String text, Point center, Point lowerLeftCorner, Point lowerRightCorner, Canvas canvas)
    {
        float cornerRound = 5 * canvasDensityScale;

        float tag_id_width = textPaint.measureText(text) + 20*canvasDensityScale;
        float tag_id_height = 50*canvasDensityScale;

        float id_x = (float) center.x * bmpPxToCanvasPx - tag_id_width/2;
        float id_y = (float) center.y * bmpPxToCanvasPx - tag_id_height/2;

        float tag_id_text_x = id_x + 10*canvasDensityScale;
        float tag_id_text_y = id_y + 40*canvasDensityScale;

        canvas.save();
        canvas.rotate((float) Math.toDegrees(Math.atan2(lowerRightCorner.y - lowerLeftCorner.y, lowerRightCorner.x-lowerLeftCorner.x)), (float) center.x*bmpPxToCanvasPx, (float) center.y*bmpPxToCanvasPx);

        canvas.drawRoundRect(id_x, id_y, id_x+tag_id_width, id_y+tag_id_height, cornerRound, cornerRound, rectPaint);
        canvas.drawText(text, tag_id_text_x, tag_id_text_y, textPaint);

        canvas.restore();
    }
}
