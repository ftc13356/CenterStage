package org.firstinspires.ftc.teamcode.SampleDetect;

import static org.opencv.calib3d.Calib3d.Rodrigues;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleDetectionPipelinePNP extends OpenCvPipeline
{
    /*
     * Our working image buffers
     */
    Mat ycrcbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();

    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat blueEdgeMat = new Mat();
    Mat redEdgeMat = new Mat();
    Mat yellowEdgeMat = new Mat();


    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    public static  int YELLOW_MASK_THRESHOLD = 57;
    public static  int BLUE_MASK_THRESHOLD = 150;

    public static int retVal = 0;

    public static int threshold1 = 50, threshold2 = 100;
    public static  int RED_MASK_THRESHOLD = 178;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    /*
     * Colors
     */
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);

    // Define the color range for filtering
    public static double H = 90, S = 50, V = 90;
    Scalar upperBlue = new Scalar(140, 255, 255);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
    Mat hsv = new Mat();
    Mat maskedImage = new Mat();
    Mat blurredImage = new Mat();
    Mat closedEdges = new Mat();
    Mat mask = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();
    Mat boundingImage = new Mat();

    public static int BLUR = 5, UPPER_THRESH = 25, LOWER_THRESH = 0, KERNEL_SIZE = 3;


    static final int CONTOUR_LINE_THICKNESS = 2;

    double[] center;

    static class AnalyzedStone
    {
        double angle;
        String color;
        Mat rvec;
        Mat tvec;
    }

    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    /*
     * Camera Calibration Parameters
     */
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage
    {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS;
    }

    Stage[] stages = Stage.values();

    // Keep track of what stage the viewport is showing
    int stageNum = 0;

    public SampleDetectionPipelinePNP()
    {
        // Initialize camera parameters
        // Replace these values with your actual camera calibration parameters

        // Focal lengths (fx, fy) and principal point (cx, cy)
        double fx = 622.001; // Replace with your camera's focal length in pixels
        double fy = 622.001;
        double cx = 319.803; // Replace with your camera's principal point x-coordinate (usually image width / 2)
        double cy = 241.251; // Replace with your camera's principal point y-coordinate (usually image height / 2)

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        // Distortion coefficients (k1, k2, p1, p2, k3)
        // If you have calibrated your camera and have these values, use them
        // Otherwise, you can assume zero distortion for simplicity
        distCoeffs = new MatOfDouble(.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0);
    }

    @Override
    public void onViewportTapped()
    {
        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Convert the image to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBlue = new Scalar(H, S, V);

        // Create a mask based on the color range
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        // Bitwise-AND mask and original image to get the colored parts

        maskedImage = new Mat();

        Core.bitwise_and(input, input, maskedImage, mask);

        blurredImage = new Mat();


        // Apply Gaussian blur to the masked image
        Imgproc.GaussianBlur(maskedImage, blurredImage, new Size(BLUR, BLUR), 0);

        edges = new Mat();
        // Apply Canny edge detection
        Imgproc.Canny(blurredImage, edges, LOWER_THRESH, UPPER_THRESH);

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));
        // Morphological close operation
        closedEdges = new Mat();
        Imgproc.morphologyEx(edges, closedEdges, Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        hierarchy = new Mat();
        Imgproc.findContours(closedEdges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        boundingImage = input.clone();

        // Set acceptable aspect ratio range
        double minAspectRatio = 3.5/1.5-0.3;
        double maxAspectRatio = 3.5/1.5+0.3;
        double minAreaThreshold = 800;  // Minimum area threshold

        // Iterate over contours
        for (MatOfPoint contour : contours) {
            // Filter out small contours based on area
            if (Imgproc.contourArea(contour) < minAreaThreshold) {
                continue;
            }

            // Approximate the contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            double epsilon = 0.08 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approx, epsilon, true);
            MatOfPoint approxMat = new MatOfPoint(approx.toArray());

            // Check if the approximated contour has 4 points (quadrilateral) and is convex
            if (approx.total() == 4 && Imgproc.isContourConvex(approxMat)) {
                RotatedRect minAreaRect = Imgproc.minAreaRect(approx);
                Point[] box = new Point[4];
                minAreaRect.points(box); // Get the rectangle's corner points

                // Calculate width and height from the rectangle
                double width = minAreaRect.size.width;
                double height = minAreaRect.size.height;

                // Calculate the aspect ratio
                if (height != 0) {  // Avoid division by zero
                    double aspectRatio = Math.max(width, height) / Math.min(width, height);

                    // Check if the aspect ratio is within the specified range
                    if (minAspectRatio <= aspectRatio && aspectRatio <= maxAspectRatio) {
                        // Draw the bounding rectangle on the image
                        for (int j = 0; j < 4; j++) {
                            Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                        }
                        drawTagText(minAreaRect, Double.toString(minAreaRect.angle), boundingImage, "Blue");
                        // Prepare object points and image points for solvePnP
                        // Assuming the object is a rectangle with known dimensions
                        double objectWidth = 3.5;  // Replace with your object's width in real-world units (e.g., centimeters)
                        double objectHeight = 1.5;  // Replace with your object's height in real-world units

                        // Define the 3D coordinates of the object corners in the object coordinate space
                        MatOfPoint3f objectPoints = new MatOfPoint3f(
                                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                                new Point3(objectWidth / 2, -objectHeight / 2, 0),
                                new Point3(objectWidth / 2, objectHeight / 2, 0),
                                new Point3(-objectWidth / 2, objectHeight / 2, 0)
                        );


                        // Order the image points in the same order as object points
                        Point[] orderedRectPoints = orderPoints(approx.toArray());

                        MatOfPoint2f imagePoints = new MatOfPoint2f(orderedRectPoints);

                        // Solve PnP
                        Mat rvec = new Mat();
                        Mat tvec = new Mat();

                        boolean success = Calib3d.solvePnP(
                                objectPoints, // Object points in 3D
                                imagePoints,  // Corresponding image points
                                cameraMatrix,
                                distCoeffs,
                                rvec,
                                tvec
                        );
                        if(success){
                            double[] coords = new double[3];
                            tvec.get(0, 0, coords);
                            this.center[0] = coords[0];
                            this.center[1] = coords[1];
                            this.center[2] = coords[2];
                            drawTagText(minAreaRect, (this.center[0]+","+this.center[1]+","+this.center[2]), boundingImage, "Blue");

                        }

                        break;
                    }
                }
            }
        }
//        input.release();
//        hsv.release();
        if(retVal==0){
            return boundingImage;
        }
        else if(retVal ==1){
            return mask;
        }
        else if(retVal ==2){
            return blurredImage;
        }
        else if(retVal ==3){
            return edges;
        }
        else if(retVal == 4){
            return closedEdges;
        }
        return boundingImage;
    }

    public ArrayList<AnalyzedStone> getDetectedStones()
    {
        return clientStoneList;
    }

    void findContours(Mat input)
    {
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb channel for blue detection
        Core.extractChannel(ycrcbMat, cbMat, 2); // Cb channel index is 2

        // Extract the Cr channel for red detection
        Core.extractChannel(ycrcbMat, crMat, 1); // Cr channel index is 1

        // Threshold the Cb channel to form a mask for blue
        Imgproc.threshold(cbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        // Threshold the Cr channel to form a mask for red
        Imgproc.threshold(crMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        // Threshold the Cb channel to form a mask for yellow
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        morphMask(blueThresholdMat, morphedBlueThreshold);


        Imgproc.Canny(input,blueEdgeMat, threshold1,threshold2);
        Imgproc.Canny(redThresholdMat,redEdgeMat, 75,175);
        Imgproc.Canny(yellowThresholdMat,yellowEdgeMat, 75,175);


        // Apply morphology to the masks
        morphMask(blueEdgeMat, morphedBlueThreshold);
        morphMask(redEdgeMat, morphedRedThreshold);
        morphMask(yellowEdgeMat, morphedYellowThreshold);

        // Find contours in the masks
        ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        Imgproc.drawContours(blueEdgeMat, blueContoursList, -1, new Scalar (0, 255, 0), 2);

        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        // Now analyze the contours
        int contourNum = 0;
        for(MatOfPoint contour : blueContoursList)
        {
            analyzeContour(contour, input, "Blue", contourNum);
            contourNum++;
        }
        contourNum = 0;
        for(MatOfPoint contour : redContoursList)
        {
            analyzeContour(contour, input, "Red",contourNum);
            contourNum++;
        }
        contourNum = 0;
        for(MatOfPoint contour : yellowContoursList)
        {
            analyzeContour(contour, input, "Yellow",contourNum);
            contourNum++;
        }
    }

    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color, int contourNum)
    {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        double epsilon = .02*Imgproc.arcLength(contour2f, true);
        MatOfPoint2f approx = new MatOfPoint2f();

        Imgproc.approxPolyDP(contour2f, approx, epsilon, true);
        if(approx.total()==4) {
            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(approx, input, color);

            // The angle OpenCV gives us can be ambiguous, so look at the shape of
            // the rectangle to fix that.
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                rotRectAngle += 90;
            }

            // Compute the angle and store it
            double angle = -(rotRectAngle - 180);
            drawTagText(rotatedRectFitToContour, Integer.toString(contourNum) + " deg", input, color);

            // Prepare object points and image points for solvePnP
            // Assuming the object is a rectangle with known dimensions
            double objectWidth = 3.5;  // Replace with your object's width in real-world units (e.g., centimeters)
            double objectHeight = 1.5;  // Replace with your object's height in real-world units

            // Define the 3D coordinates of the object corners in the object coordinate space
            MatOfPoint3f objectPoints = new MatOfPoint3f(
                    new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                    new Point3(objectWidth / 2, -objectHeight / 2, 0),
                    new Point3(objectWidth / 2, objectHeight / 2, 0),
                    new Point3(-objectWidth / 2, objectHeight / 2, 0)
            );

            // Get the 2D image points from the detected rectangle corners
            Point[] rectPoints = new Point[4];
            rotatedRectFitToContour.points(rectPoints);

            // Order the image points in the same order as object points
            Point[] orderedRectPoints = orderPoints(approx.toArray());

            MatOfPoint2f imagePoints = new MatOfPoint2f(orderedRectPoints);

            // Solve PnP
            Mat rvec = new Mat();
            Mat tvec = new Mat();

            boolean success = Calib3d.solvePnP(
                    objectPoints, // Object points in 3D
                    imagePoints,  // Corresponding image points
                    cameraMatrix,
                    distCoeffs,
                    rvec,
                    tvec
            );

            if (success) {
                // Draw the coordinate axes on the image
                drawAxis(input, rvec, tvec, cameraMatrix, distCoeffs);

                // Store the pose information
                AnalyzedStone analyzedStone = new AnalyzedStone();
                analyzedStone.angle = rotRectAngle;
                analyzedStone.color = color;
                analyzedStone.rvec = rvec;
                analyzedStone.tvec = tvec;
                internalStoneList.add(analyzedStone);
            }
        }
    }

    void drawAxis(Mat img, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs)
    {
        // Length of the axis lines
        double axisLength = 5.0;

        // Define the points in 3D space for the axes
        MatOfPoint3f axisPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(axisLength, 0, 0),
                new Point3(0, axisLength, 0),
                new Point3(0, 0, -axisLength) // Z axis pointing away from the camera
        );

        // Project the 3D points to 2D image points
        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();

        // Draw the axis lines
        Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 2); // X axis in red
        Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 2); // Y axis in green
        Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 2); // Z axis in blue
    }

    static Point[] orderPoints(Point[] pts)
    {
        // Orders the array of 4 points in the order: top-left, top-right, bottom-right, bottom-left
        Point[] orderedPts = new Point[4];

        // Sum and difference of x and y coordinates
        double[] sum = new double[4];
        double[] diff = new double[4];

        for (int i = 0; i < 4; i++)
        {
            sum[i] = pts[i].x + pts[i].y;
            diff[i] = pts[i].y - pts[i].x;
        }

        // Top-left point has the smallest sum
        int tlIndex = indexOfMin(sum);
        orderedPts[0] = pts[tlIndex];

        // Bottom-right point has the largest sum
        int brIndex = indexOfMax(sum);
        orderedPts[2] = pts[brIndex];

        // Top-right point has the smallest difference
        int trIndex = indexOfMin(diff);
        orderedPts[1] = pts[trIndex];

        // Bottom-left point has the largest difference
        int blIndex = indexOfMax(diff);
        orderedPts[3] = pts[blIndex];

        return orderedPts;
    }

    static int indexOfMin(double[] array)
    {
        int index = 0;
        double min = array[0];

        for (int i = 1; i < array.length; i++)
        {
            if (array[i] < min)
            {
                min = array[i];
                index = i;
            }
        }
        return index;
    }

    static int indexOfMax(double[] array)
    {
        int index = 0;
        double max = array[0];

        for (int i = 1; i < array.length; i++)
        {
            if (array[i] > max)
            {
                max = array[i];
                index = i;
            }
        }
        return index;
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color)
    {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                colorScalar, // Font color
                1); // Font thickness
    }

    static void drawRotatedRect(MatOfPoint2f approx, Mat drawOn, String color)
    {
        Point[] points2f = approx.toArray();
        Point[] points = new Point[points2f.length];

        for (int i = 0; i < points2f.length; i++) {
            points[i] = new Point(Math.round(points2f[i].x), Math.round(points2f[i].y)); // Convert to integer
        }

        // Create MatOfPoint from the Point array
        MatOfPoint quadrilateral = new MatOfPoint(points);

        // Convert MatOfPoint to a list for polylines
        List<MatOfPoint> quadrilaterals = new ArrayList<>();
        quadrilaterals.add(quadrilateral);

        Scalar colorScalar = getColorScalar(color);

        // Draw the quadrilateral on the input image
        Imgproc.polylines(drawOn, quadrilaterals, true, colorScalar, 2); // Red color, thickness 2
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

//        Point[] points = new Point[4];
//        rect.points(points);
//
//        Scalar colorScalar = getColorScalar(color);
//
//        for (int i = 0; i < 4; ++i)
//        {
//            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
//        }
    }
    public double[] getCenter(){
        return center;
    }

    static Scalar getColorScalar(String color)
    {
        switch (color)
        {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }
}