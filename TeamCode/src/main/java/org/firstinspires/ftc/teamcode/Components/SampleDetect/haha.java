package org.firstinspires.ftc.teamcode.Components.SampleDetect;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class haha extends OpenCvPipeline {

    private final Mat hsvMat = new Mat();
    private final Mat lowerRedMask = new Mat();
    private final Mat upperRedMask = new Mat();
    private final Mat thresholdMat = new Mat();
    private final Mat edgesMat = new Mat();
    private final Mat dilatedMat = new Mat();
    private final Mat hierarchy = new Mat();

    // HSV range for red (split into two ranges)
    private static final Scalar RED_LOWER1 = new Scalar(0, 150, 50);   // Lower red range
    private static final Scalar RED_UPPER1 = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER2 = new Scalar(170, 150, 50); // Upper red range
    private static final Scalar RED_UPPER2 = new Scalar(180, 255, 255);

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold for red (two ranges)
        Core.inRange(hsvMat, RED_LOWER1, RED_UPPER1, lowerRedMask);
        Core.inRange(hsvMat, RED_LOWER2, RED_UPPER2, upperRedMask);

        Core.addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, thresholdMat);

        // Edge detection using Canny
        Imgproc.Canny(thresholdMat, edgesMat, 50, 150);

        // Dilate edges to strengthen contours
        Imgproc.dilate(edgesMat, dilatedMat, new Mat(), new Point(-1, -1), 2);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(dilatedMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Process contours
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            // Filter small contours
            if (area > 500) {
                // Get bounding box
                RotatedRect boundingBox = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                double aspectRatio = Math.max(boundingBox.size.width, boundingBox.size.height) /
                        Math.min(boundingBox.size.width, boundingBox.size.height);
                if (aspectRatio >= 2.3 && aspectRatio <= 2.6) { // Allow some variation
                    // Draw bounding box
                    Point[] boxPoints = new Point[4];
                    boundingBox.points(boxPoints);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                    }
                }
            }
        }

        return input; // Return the processed frame with contours drawn
    }
}
