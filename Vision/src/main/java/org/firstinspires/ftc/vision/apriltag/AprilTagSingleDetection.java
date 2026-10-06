/*
 * Copyright (c) 2026 FIRST
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Point;

/**
 * This class represents the detection of a SINGLE AprilTag, that is not part of a cluster
 */
public class AprilTagSingleDetection extends AprilTagDetection
{
    /**
     * The numerical ID of the detection
     */
    public final int id;

    /**
     * The number of bits corrected when reading the tag ID payload
     */
    public final int hamming;

    /*
     * How much margin remains before the detector would decide to reject a tag
     */
    public final float decisionMargin;

    /*
     * The image pixel coordinates of the corners of the tag
     */
    public final Point[] corners;

    /*
     * The image pixel coordinates of the center of the tag
     */
    public final Point center;

    /*
     * Metadata known about this tag from the tag library set on the detector;
     * will be NULL if the tag was not in the tag library
     */
    public final AprilTagMetadata metadata;

    public AprilTagSingleDetection(int id, int hamming, float decisionMargin, Point center, Point[] corners,
                                   AprilTagMetadata metadata, AprilTagPoseFtc ftcPose, AprilTagPoseRaw rawPose,
                                   Pose3D robotPose, long frameAcquisitionNanoTime, DistanceUnit distanceUnit)
    {
        super(ftcPose, rawPose, robotPose, frameAcquisitionNanoTime,distanceUnit);

        this.id = id;
        this.hamming = hamming;
        this.decisionMargin = decisionMargin;
        this.metadata = metadata;
        this.corners = corners;
        this.center = center;
    }
}
