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

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.ArrayList;

public class AprilTagClusterMetadata
{
    public final String name;
    public final String shortName;
    public final DistanceUnit distanceUnit;
    public final VectorF fieldPosition;
    public final Quaternion fieldOrientation;

    // Cannot seed these at 0 else 0,0 will be included
    // in the extents when the origin is outside all the tags
    double xExtentMin = Double.POSITIVE_INFINITY;
    double xExtentMax = Double.NEGATIVE_INFINITY;
    double yExtentMin = Double.POSITIVE_INFINITY;
    double yExtentMax = Double.NEGATIVE_INFINITY;
    double zExtentMin = Double.POSITIVE_INFINITY;
    double zExtentMax = Double.NEGATIVE_INFINITY;
    double largestTagSize = 0;
    ArrayList<AprilTagClusterMemberMetadata> clusterMembers;

    /**
     * Add a tag cluster to this tag library
     * @param clusterMembers the members of this cluster
     * @param name a text name for the tag
     * @param fieldPosition a vector describing the tag's 3d translation on the field
     * @param distanceUnit the units used for size and fieldPosition
     * @param fieldOrientation a quaternion describing the tag's orientation on the field
     */
    public AprilTagClusterMetadata(ArrayList<AprilTagClusterMemberMetadata> clusterMembers, String name, String shortName,
                                   VectorF fieldPosition, DistanceUnit distanceUnit, Quaternion fieldOrientation)
    {
        if (clusterMembers.size() < 2)
        {
            throw new IllegalArgumentException("Cannot make a cluster with less than 2 tags");
        }

        for (int i = 0; i < clusterMembers.size(); i++)
        {
            for (int j = 0; j < clusterMembers.size(); j++)
            {
                if (i == j)
                {
                    continue;
                }

                if (clusterMembers.get(i).id == clusterMembers.get(j).id)
                {
                    throw new IllegalArgumentException("Cannot create cluster with duplicate tag IDs");
                }
            }
        }

        // Autocalculate the extent of the outer edges of the tags
        for (AprilTagClusterMemberMetadata m : clusterMembers)
        {
            xExtentMin = Math.min(xExtentMin, m.positionInClusterPlane.get(0) - m.tagsize / 2);
            xExtentMax = Math.max(xExtentMax, m.positionInClusterPlane.get(0) + m.tagsize / 2);
            yExtentMin = Math.min(yExtentMin, m.positionInClusterPlane.get(1) - m.tagsize / 2);
            yExtentMax = Math.max(yExtentMax, m.positionInClusterPlane.get(1) + m.tagsize / 2);
            zExtentMin = Math.min(zExtentMin, m.positionInClusterPlane.get(2));
            zExtentMax = Math.max(zExtentMax, m.positionInClusterPlane.get(2));

            largestTagSize = Math.max(largestTagSize, m.tagsize);
        }

        this.clusterMembers = clusterMembers;
        this.name = name;
        this.shortName = shortName;
        this.distanceUnit = distanceUnit;
        this.fieldPosition = fieldPosition;
        this.fieldOrientation = fieldOrientation;
    }

    // internal use
    protected boolean containsTagId(int id)
    {
        for (AprilTagClusterMemberMetadata m : clusterMembers)
        {
            if (m.id == id)
            {
                return true;
            }
        }

        return false;
    }

    // internal use
    protected ArrayList<Integer> getMemberIds()
    {
        ArrayList<Integer> ids = new ArrayList<>();

        for (AprilTagClusterMemberMetadata m : clusterMembers)
        {
            ids.add(m.id);
        }

        return ids;
    }

    // internal use
    protected AprilTagClusterMemberMetadata getMemberMetadata(int id)
    {
        for (AprilTagClusterMemberMetadata m : clusterMembers)
        {
            if (m.id == id)
            {
                return m;
            }
        }

        return null;
    }
}
