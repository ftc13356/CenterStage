package org.firstinspires.ftc.teamcode.SampleDetect;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@Config
public class BluePipeline extends OpenCvPipeline {
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
    public static int YELLOW_MASK_THRESHOLD = 57;
    public static int BLUE_MASK_THRESHOLD = 150;

    public static int retVal = 0;

    public static int threshold1 = 0, threshold2 = 60;
    public static int RED_MASK_THRESHOLD = 178;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    public static double DISTANCe = 7.77;

    List<MatOfPoint> contours = new ArrayList<>();

    /*
     * Colors
     */
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);

    // Define the color range for filtering
    public static double H = 100, S = 70, V = 70;
    Scalar upperBlue = new Scalar(150, 255, 255);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
    Mat hsv = new Mat();
    Mat maskedImage = new Mat();
    Mat blurredImage = new Mat();
    Mat closedEdges = new Mat();
    Mat mask = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();
    Mat boundingImage = new Mat();

    public static int BLUR = 1, CLAHE = 1, UPPER_THRESH = 130, LOWER_THRESH = 130, KERNEL_SIZE = 2;
    public static double AREA_THRESH = .85, BYPASS_RATIO = 0.1, FCL = 1, EPSILON = 0.04, UP_TOLERANCE = 0.5, DOWN_TOLERANCE = 0.5, CLASSUP_TOL = 0.8, CLASSDOWN_TOL = 0.7, PLUS_ULTRA = 0.5,UP_TOLERANCE2 =2, DOWN_TOLERANCE2 =1.25;

    // Prepare object points and image points for solvePnP
    // Assuming the object is a rectangle with known dimensions
    double objectWidth = 3.5;  // Replace with your object's width in real-world units (e.g., centimeters)
    double objectHeight = 1.5;  // Replace with your object's height in real-world units

    // Define the 3D coordinates of the object corners in the object coordinate space
    MatOfPoint3f objectPoints = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));
    MatOfPoint3f objectPoints1 = new MatOfPoint3f(
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, -1),
            new Point3(objectWidth / 2, -objectHeight / 2, -1),
            new Point3(objectWidth / 2, objectHeight / 2, 0));
    MatOfPoint3f objectPoints2 = new MatOfPoint3f(
            new Point3(objectWidth / 2, -objectHeight / 2, -1),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(objectWidth / 2, objectHeight / 2, -1)
    );

    Point[] orderedRectPoints;
    Mat rvec = new Mat();
    Mat tvec = new Mat();
    MatOfPoint2f imagePoints = new MatOfPoint2f(), contour2f = new MatOfPoint2f(), approx = new MatOfPoint2f();
    MatOfPoint approxMat;


    static final int CONTOUR_LINE_THICKNESS = 2;

    double[] center = {0, 0, 0};

    static class AnalyzedStone {
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
    enum Stage {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS;
    }

    Stage[] stages = Stage.values();

    // Keep track of what stage the viewport is showing
    int stageNum = 0;
    RotatedRect minAreaRect;

    public BluePipeline() {
        // Initialize camera parameters
        // Replace these values with your actual camera calibration parameters

        // Focal lengths (fx, fy) and principal point (cx, cy)
        // Focals (pixels) - Fx: 396.39 Fy: 396.39
        //Optical center - Cx: 646.424 Cy: 340.884
        //Radial distortion (Brown's Model)
        //K1: 0.00981454 K2: -0.0292495 K3: 0.00489965
        //P1: -0.000205308 P2: -4.06933e-05
        double fx = 396.39 * FCL; // Replace with your camera's focal length in pixels
        double fy = 396.39 * FCL;
        double cx = 646.424; // Replace with your camera's principal point x-coordinate (usually image width / 2)
        double cy = 360.884; // Replace with your camera's principal point y-coordinate (usually image height / 2)

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        // Distortion coefficients (k1, k2, p1, p2, k3)
        // If you have calibrated your camera and have these values, use them
        // Otherwise, you can assume zero distortion for simplicity
        distCoeffs = new MatOfDouble(0.00981454, -0.0292495, 0.00489965, -0.000205308, -4.06933e-05);
    }

    @Override
    public void onViewportTapped() {
        int nextStageNum = stageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    public void resetCenter() {
        center = new double[]{0, 0, 0};
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert the image to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBlue = new Scalar(H, S, V);

        // Create a mask based on the color range
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        // Bitwise-AND mask and original image to get the colored parts

//        maskedImage = new Mat();
//
//        Core.bitwise_and(input, input, maskedImage, mask);
//
//        blurredImage = new Mat();
//
//
//        // Apply Gaussian blur to the masked image
//        Imgproc.GaussianBlur(maskedImage, blurredImage, new Size(BLUR, BLUR), 0);
//        Mat grayBlur = new Mat();
//        Imgproc.cvtColor(blurredImage, grayBlur, Imgproc.COLOR_BGR2GRAY);
//        Mat claheIm = new Mat();
//        CLAHE clahe = Imgproc.createCLAHE();
//        clahe.setClipLimit(CLAHE);
//        clahe.apply(grayBlur, claheIm);
//
//
//        edges = new Mat();
//        // Apply Canny edge detection
//        Imgproc.Canny(claheIm, edges, LOWER_THRESH, UPPER_THRESH);
//
//        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(KERNEL_SIZE, KERNEL_SIZE));
////        // Morphological close operation
//        closedEdges = new Mat();
//        Imgproc.dilate(edges, closedEdges, kernel);
//        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));
//        Imgproc.morphologyEx(closedEdges, edges,Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        boundingImage = input.clone();

        // Set acceptable aspect ratio range
        double minAspectRatio = 3.5 / 1.5 - DOWN_TOLERANCE;
        double maxAspectRatio = 3.5 / 1.5 + UP_TOLERANCE;
        double minAspectRatio2 = 3.5 / 1.5 - DOWN_TOLERANCE2;
        double maxAspectRatio2 = 3.5 / 1.5 + UP_TOLERANCE2;
        double minAreaThreshold = 10000;  // Minimum area threshold
//        Imgproc.drawContours(boundingImage, contours, -1,new Scalar(255,0,0),2);

        // Iterate over contours
        for (MatOfPoint contour : contours) {
            // Filter out small contours based on area
            if (Imgproc.contourArea(contour) < minAreaThreshold) {
                continue;
            }

            // Approximate the contour to a polygon
            contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = EPSILON * Imgproc.arcLength(contour2f, true);
            approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, epsilon, true);
            approxMat = new MatOfPoint(approx.toArray());
            minAreaRect = Imgproc.minAreaRect(contour2f);

            // Check if the approximated contour has 4 points (quadrilateral) and is convex
            if (approx.total() == 4 && Imgproc.isContourConvex(approxMat)) {
//                minAreaRect.points(box); // Get the rectangle's corner points

                // Calculate width and height from the rectangle

                Point[] orded = orderPoints(approx.toArray());
                double[] distances = {distance(orded[0], orded[1]), distance(orded[1], orded[2]), distance(orded[0], orded[2])};
                Arrays.sort(distances);
                double width = distances[1];
                double height = distances[0];
                double expectedD = width * width + height * height;
                double actD = distances[2] * distances[2];


                // Calculate the aspect ratio
                if (height != 0) {  // Avoid division by zero
                    double aspectRatio = width / height;
                    packet.put("aspecticor", aspectRatio);
                    // Check if the aspect ratio is within the specified range
                    if (minAspectRatio <= aspectRatio && aspectRatio <= maxAspectRatio && abs((actD - expectedD) / expectedD) < BYPASS_RATIO) {
                        // Draw the bounding rectangle on the image
                        for (int j = 0; j < 4; j++) {
                            Imgproc.line(boundingImage, approx.toArray()[j], approx.toArray()[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                        }
                        double rotRectAngle = minAreaRect.angle;
                        if (minAreaRect.size.width < minAreaRect.size.height) {
                            rotRectAngle += 90;
                        }

                        // Compute the angle and store it
                        double angle = -(90 + rotRectAngle - 180);
                        drawTagText(minAreaRect, angle + " deg", boundingImage, "Blue");


                        // Order the image points in the same order as object points
                        orderedRectPoints = orderPoints(approx.toArray());


                        // Solve PnP
//                        rvec = new Mat();
//                        tvec = new Mat();

                        MatOfPoint3f rlObjectPoints = new MatOfPoint3f();
                        int classify = 0;
                        if (aspectRatio < 3.5 / 1.5 - CLASSDOWN_TOL) {
                            classify = 1;
                        } else if (aspectRatio > 3.5 / 1.5 + CLASSUP_TOL) {
                            classify = 2;
                        }
                        if (classify == 2) {
                            rlObjectPoints = objectPoints2;
//                            Point[] buh = order4(orderedRectPoints);
//                            if (!buh.equals(orderedRectPoints)) {
//                                classify = 4;
//                                orderedRectPoints = buh;
//                                rlObjectPoints = objectPoints3;
//                            }
                        } else if (classify == 1) {
                            rlObjectPoints = objectPoints1;
//                            Point[] buh = order4(orderedRectPoints);
//                            if (!buh.equals(orderedRectPoints)) {
//                                classify = 4;
//                                orderedRectPoints = buh;
//                                rlObjectPoints = objectPoints3;
//                            }
                        } else {
                            rlObjectPoints = objectPoints;
                            orderedRectPoints = orderCorner(orderedRectPoints);
//                            Point[] buh = order4(orderedRectPoints);
//                            if (!buh.equals(orderedRectPoints)) {
//                                classify = 4;
//                                orderedRectPoints = buh;
//                                rlObjectPoints = objectPoints3;
//                            }
                        }
                        imagePoints = new MatOfPoint2f(orderedRectPoints);

                        boolean success = Calib3d.solvePnP(
                                rlObjectPoints, // Object points in 3D
                                imagePoints,  // Corresponding image points
                                cameraMatrix,
                                distCoeffs,
                                rvec,
                                tvec
                        );
                        if (success) {
                            double[] coords = new double[3];
                            tvec.get(0, 0, coords);
                            tvec.get(0, 0, coords);
                            this.center[1] = coords[1];
                            this.center[2] = coords[2];
//                            double expectH = DISTANCe;
//                            if(classify != 0){
//                                expectH += PLUS_ULTRA;
//                            }
//                            double consta = expectH/ coords[2];
                            double consta = 1 /** (Imgproc.contourArea(contour)/minAreaRect.size.height * minAreaRect.size.width)*/;
                            center[0] = coords[0] * consta;
                            center[1] = coords[1] * consta;
                            center[2] = coords[2] * consta;

                            packet.put("CAM X", coords[0]);
                            packet.put("CAM y", coords[1]);
                            packet.put("CAM Z", coords[2]);
                            packet.put("classif", classify);
                            packet.put("aspectRatio", aspectRatio);
                            packet.put("area", Imgproc.contourArea(contour));

//                            drawTagText(minAreaRect, (test), boundingImage, "Red");

                        } else {
                        }
                        break;
                    } else if (minAreaRect.size.width != 0 && minAreaRect.size.height != 0 && Imgproc.contourArea(contour) / (minAreaRect.size.height * minAreaRect.size.width) > AREA_THRESH) {
// Calculate width and height from the rectangle
                        Point[] box = new Point[4];
                        minAreaRect.points(box);
                        orded = orderPoints(box);
                        distances = new double[]{distance(orded[0], orded[1]), distance(orded[1], orded[2]), distance(orded[0], orded[2])};
                        Arrays.sort(distances);
                        width = distances[1];
                        height = distances[0];

                        // Calculate the aspect ratio
                        if (height != 0) {  // Avoid division by zero
                            aspectRatio = width / height;

                            // Check if the aspect ratio is within the specified range
                            if (minAspectRatio2 <= aspectRatio && aspectRatio <= maxAspectRatio2) {
                                // Draw the bounding rectangle on the image
                                for (int j = 0; j < 4; j++) {
                                    Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                                }
                                double rotRectAngle = minAreaRect.angle;
                                if (minAreaRect.size.width < minAreaRect.size.height) {
                                    rotRectAngle += 90;
                                }

                                // Compute the angle and store it
                                double angle = -(90 + rotRectAngle - 180);
                                drawTagText(minAreaRect, angle + " deg", boundingImage, "Blue");


                                // Order the image points in the same order as object points
                                orderedRectPoints = orderPoints(box);


                                // Solve PnP
//                        rvec = new Mat();
//                        tvec = new Mat();

                                MatOfPoint3f rlObjectPoints = new MatOfPoint3f();
                                int classify = 0;
                                if (aspectRatio < 3.5 / 1.5 - CLASSDOWN_TOL) {
                                    classify = 1;
                                } else if (aspectRatio > 3.5 / 1.5 + CLASSUP_TOL) {
                                    classify = 2;
                                }
                                if (classify == 2) {
                                    rlObjectPoints = objectPoints2;
                                } else if (classify == 1) {
                                    rlObjectPoints = objectPoints1;
                                } else {
                                    rlObjectPoints = objectPoints;
                                    orderedRectPoints = orderCorner(orderedRectPoints);
                                }
                                imagePoints = new MatOfPoint2f(orderedRectPoints);

                                boolean success = Calib3d.solvePnP(
                                        rlObjectPoints, // Object points in 3D
                                        imagePoints,  // Corresponding image points
                                        cameraMatrix,
                                        distCoeffs,
                                        rvec,
                                        tvec
                                );
                                if (success) {
                                    double[] coords = new double[3];
                                    tvec.get(0, 0, coords);
                                    tvec.get(0, 0, coords);
                                    this.center[1] = coords[1];
                                    this.center[2] = coords[2];
//                            double expectH = DISTANCe;
//                            if(classify != 0){
//                                expectH += PLUS_ULTRA;
//                            }
//                            double consta = expectH/ coords[2];
                                    double multiplia = sqrt(aspectRatio/(3.5/1.5));
                                    if(multiplia<1){
                                        multiplia = 1;
                                    }
                                    double consta = 1.16* Imgproc.contourArea(contour)/(minAreaRect.size.height * minAreaRect.size.width)*multiplia;                                    center[0] = coords[0] * consta;
                                    center[1] = coords[1] * consta;
                                    center[2] = coords[2] * consta;

                                    packet.put("CAM X", center[0]);
                                    packet.put("CAM y", center[1]);
                                    packet.put("CAM Z", center[2]);
                                    packet.put("classif", classify);
                                    packet.put("aspectRatio", aspectRatio);
                                    packet.put("area", Imgproc.contourArea(contour));

//                            drawTagText(minAreaRect, (test), boundingImage, "Red");

                                } else {
                                }
                                break;
                            }
                        }
                    }

                }
            } else if (minAreaRect.size.width != 0 && minAreaRect.size.height != 0 && Imgproc.contourArea(contour) / (minAreaRect.size.height * minAreaRect.size.width) > AREA_THRESH) {
// Calculate width and height from the rectangle
                Point[] box = new Point[4];
                minAreaRect.points(box);
                Point[] orded = orderPoints(box);
                double[] distances = {distance(orded[0], orded[1]), distance(orded[1], orded[2]), distance(orded[0], orded[2])};
                Arrays.sort(distances);
                double width = distances[1];
                double height = distances[0];

                // Calculate the aspect ratio
                if (height != 0) {  // Avoid division by zero
                    double aspectRatio = width / height;

                    // Check if the aspect ratio is within the specified range
                    if (minAspectRatio2 <= aspectRatio && aspectRatio <= maxAspectRatio2) {
                        // Draw the bounding rectangle on the image
                        for (int j = 0; j < 4; j++) {
                            Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                        }
                        double rotRectAngle = minAreaRect.angle;
                        if (minAreaRect.size.width < minAreaRect.size.height) {
                            rotRectAngle += 90;
                        }

                        // Compute the angle and store it
                        double angle = -(90 + rotRectAngle - 180);
                        drawTagText(minAreaRect, angle + " deg", boundingImage, "Blue");


                        // Order the image points in the same order as object points
                        orderedRectPoints = orderPoints(box);


                        // Solve PnP
//                        rvec = new Mat();
//                        tvec = new Mat();

                        MatOfPoint3f rlObjectPoints = new MatOfPoint3f();
                        int classify = 0;
                        if (aspectRatio < 3.5 / 1.5 - CLASSDOWN_TOL) {
                            classify = 1;
                        } else if (aspectRatio > 3.5 / 1.5 + CLASSUP_TOL) {
                            classify = 2;
                        }
                        if (classify == 2) {
                            rlObjectPoints = objectPoints2;
                        } else if (classify == 1) {
                            rlObjectPoints = objectPoints1;
                        } else {
                            rlObjectPoints = objectPoints;
                            orderedRectPoints = orderCorner(orderedRectPoints);
//                            Point[] buh = order4(orderedRectPoints);
//                            if (!buh.equals(orderedRectPoints)) {
//                                classify = 4;
//                                orderedRectPoints = buh;
//                                rlObjectPoints = objectPoints3;
//                            }
                        }
                        imagePoints = new MatOfPoint2f(orderedRectPoints);

                        boolean success = Calib3d.solvePnP(
                                rlObjectPoints, // Object points in 3D
                                imagePoints,  // Corresponding image points
                                cameraMatrix,
                                distCoeffs,
                                rvec,
                                tvec
                        );
                        if (success) {
                            double[] coords = new double[3];
                            tvec.get(0, 0, coords);
                            tvec.get(0, 0, coords);
                            this.center[1] = coords[1];
                            this.center[2] = coords[2];
//                            double expectH = DISTANCe;
//                            if(classify != 0){
//                                expectH += PLUS_ULTRA;
//                            }
//                            double consta = expectH/ coords[2];
                            double multiplia = sqrt(aspectRatio/(3.5/1.5));
                            if(multiplia<1){
                                multiplia = 1/multiplia;
                            }
                            double consta = 1.16* Imgproc.contourArea(contour)/(minAreaRect.size.height * minAreaRect.size.width)*multiplia;
//                            (minAreaRect.size.height * minAreaRect.size.width / Imgproc.contourArea(contour));
                            center[0] = coords[0] * consta;
                            center[1] = coords[1] * consta;
                            center[2] = coords[2] * consta;

                            packet.put("CAM X", center[0]);
                            packet.put("CAM y", center[1]);
                            packet.put("CAM Z", center[2]);
                            packet.put("classif", classify);
                            packet.put("aspectRatio", aspectRatio);
                            packet.put("area", Imgproc.contourArea(contour));

//                            drawTagText(minAreaRect, (test), boundingImage, "Red");

                        } else {
                        }
                        break;
                    }
                }
            }
        }
//        contours.clear();
//        input.release();
//        hsv.release();
//        boundingImage.release();
//        mask.release();
//        edges.release();
//        blurredImage.release();
//        hsv.release();
//        input.release();
        if (retVal == 0) {
            return boundingImage;
        } else if (retVal == 1) {
            return mask;
        } else if (retVal == 2) {
            return blurredImage;
        } else if (retVal == 3) {
            return edges;
        } else if (retVal == 4) {
            return closedEdges;
        } else if (retVal == 5) {
            return input;
        }
        return boundingImage;
    }


    // Thresholds for noise handling
    public static double DISTANCE_THRESHOLD = 15.0; // Adjust based on your needs
    public static double SLOPE_THRESHOLD = 0.08; // Adjust for slope similarity


    public int classifyShape(Point[] points) {
        if (points.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }

        // Sort points in counter-clockwise order
        points = orderPoints(points);

        // Now points are ordered as A, B, C, D
        Point A = points[0];
        Point B = points[1];
        Point C = points[2];
        Point D = points[3];

        // Calculate distances
        double AB = distance(A, B);
        double BC = distance(B, C);
        double CD = distance(C, D);
        double DA = distance(D, A);

        // Check for parallelogram
//        if (Math.abs(AB - CD) < DISTANCE_THRESHOLD && Math.abs(DA - BC) < DISTANCE_THRESHOLD) {
//            return 0;
//        }

        // Calculate slopes
        Double slopeAB = calculateSlope(A, B);
        Double slopeCD = calculateSlope(C, D);
        Double slopeAD = calculateSlope(A, D);
        Double slopeBC = calculateSlope(B, C);

        // Check if AB and CD are parallel
        boolean isParallelAB_CD = slopeAB != null && slopeCD != null && abs(slopeAB - slopeCD) < SLOPE_THRESHOLD;

        // Check if AD and BC are parallel
        boolean isParallelAD_BC = slopeAD != null && slopeBC != null && abs(slopeAD - slopeBC) < SLOPE_THRESHOLD;

        if (!isParallelAB_CD) {
            // Determine which is the long side
            if (AB > BC) {
                return 2; // AB is the long side, top left bottom right
            } else {
                return 1; // CD is the long side, top front bottom back
            }
        }
        if (!isParallelAD_BC) {
            // Determine which is the long side
            if (AB > BC) {
                return 1; // AB is the long side, top left bottom right
            } else {
                return 2; // CD is the long side, top front bottom back
            }
        }

        return 0;
    }

    private static double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }

    private static Double calculateSlope(Point p1, Point p2) {
        if (p2.x == p1.x) {
            return null; // Vertical line, slope is undefined
        }
        return (double) (p2.y - p1.y) / (p2.x - p1.x);
    }
    public static Point[] orderCorner(Point[] pts){
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }
        Point[] sortedByDistance={pts[0], null, null};
        double d1 = distance(pts[0],pts[1]);
        double d2 = distance(pts[0], pts[2]);
        double d3 = distance(pts[0], pts[3]);
        Point[] orderedPts = new Point[4];
        if(d1<d2 && d1 < d3){
            //  3 0 1 2, 3 1 0 2, 2 1 0 3, 2 0 1 3
            int[] shorts = {0,1};
            int[] longs = {2,3};
            if(isCC(pts[longs[0]], pts[shorts[0]],pts[shorts[1]],pts[longs[1]])){
                orderedPts = new Point[]{pts[longs[0]],pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            }
            else if(isCC(pts[longs[1]], pts[shorts[0]],pts[shorts[1]],pts[longs[0]])){
                orderedPts = new Point[]{pts[longs[1]],pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            }
            else if(isCC(pts[longs[0]], pts[shorts[1]],pts[shorts[0]],pts[longs[1]])){
                orderedPts = new Point[]{pts[longs[0]],pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            }
            else {
                orderedPts = new Point[]{pts[longs[1]],pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        }
        else if (d2<d3) {
            int[] shorts = {0,2};
            int[] longs = {1,3};
            if(isCC(pts[longs[0]], pts[shorts[0]],pts[shorts[1]],pts[longs[1]])){
                orderedPts = new Point[]{pts[longs[0]],pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            }
            else if(isCC(pts[longs[1]], pts[shorts[0]],pts[shorts[1]],pts[longs[0]])){
                orderedPts = new Point[]{pts[longs[1]],pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            }
            else if(isCC(pts[longs[0]], pts[shorts[1]],pts[shorts[0]],pts[longs[1]])){
                orderedPts = new Point[]{pts[longs[0]],pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            }
            else {
                orderedPts = new Point[]{pts[longs[1]],pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        }
        else {
            int[] shorts = {0,3};
            int[] longs = {1,2};
            if(isCC(pts[longs[0]], pts[shorts[0]],pts[shorts[1]],pts[longs[1]])){
                orderedPts = new Point[]{pts[longs[0]],pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            }
            else if(isCC(pts[longs[1]], pts[shorts[0]],pts[shorts[1]],pts[longs[0]])){
                orderedPts = new Point[]{pts[longs[1]],pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            }
            else if(isCC(pts[longs[0]], pts[shorts[1]],pts[shorts[0]],pts[longs[1]])){
                orderedPts = new Point[]{pts[longs[0]],pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            }
            else {
                orderedPts = new Point[]{pts[longs[1]],pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        }


        return orderedPts;
    }

    public static Point[] orderPoints(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }

        // Calculate the center of the frame
        double centerX = 640;
        double centerY =360;
        Point center = new Point((int) centerX, (int) centerY);

        // Calculate distances from each point to the center
        double[] distances = new double[4];
        for (int i = 0; i < 4; i++) {
            distances[i] = distance(center, pts[i]);
        }

        // Sort points by proximity to center
        Point[] sortedByDistance = Arrays.copyOf(pts, 4);
        Arrays.sort(sortedByDistance, Comparator.comparingDouble(p -> distance(center, p)));

        // Start with the two closest points
        Point[] orderedPts = new Point[4];
        orderedPts[0] = sortedByDistance[0]; // Closest point
        orderedPts[1] = sortedByDistance[1]; // Second closest point

        // Remaining points
        Point thirdPoint = sortedByDistance[2];
        Point fourthPoint = sortedByDistance[3];

        // Determine the order of the remaining points in clockwise manner
        if (isCC(thirdPoint, orderedPts[0], orderedPts[1],fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[0], orderedPts[1],fourthPoint};
        } else if(isCC(thirdPoint, orderedPts[1],orderedPts[0], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[1],orderedPts[0], fourthPoint};

        } else if(isCC(fourthPoint, orderedPts[1],orderedPts[0], thirdPoint)) {
            orderedPts = new Point[]{fourthPoint, orderedPts[1],orderedPts[0], thirdPoint};

        }
        else{
            orderedPts = new Point[]{fourthPoint, orderedPts[0],orderedPts[1], thirdPoint};

        }

        return orderedPts;
    }

    // Helper method to determine if three points are in clockwise order
//    public static boolean isCC(Point pt1, Point pt2, Point pt3, Point pt4) {
//        Point[] pts = new Point[] { pt1, pt2, pt3, pt4};
//        // Calculate the signed area using the shoelace formula
//        double area = 0;
//        for (int i = 0; i < 4; i++) {
//            Point p1 = pts[i];
//            Point p2 = pts[(i + 1) % 4]; // Wrap around using modulus
//            area += (p2.x - p1.x) * (p2.y + p1.y);
//        }
//
//        return area > 0; // Positive area indicates counterclockwise order
//    }
    public static boolean isCC(Point p1, Point p2, Point p3, Point p4) {
        return isCCW(p1, p2, p3) && isCCW(p1, p3, p4) &&
                isCCW(p2, p3, p4) && isCCW(p1, p2, p4);
    }

    private static boolean isCCW(Point a, Point b, Point c) {
        // Calculate the cross product of vector AB and AC
        double crossProduct = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return crossProduct > 0; // Returns true if the points are in counter-clockwise order
    }

    static int indexOfMin(double[] array) {
        int index = 0;
        double min = array[0];

        for (int i = 1; i < array.length; i++) {
            if (array[i] < min) {
                min = array[i];
                index = i;
            }
        }
        return index;
    }

    static int indexOfMax(double[] array) {
        int index = 0;
        double max = array[0];

        for (int i = 1; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                index = i;
            }
        }
        return index;
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
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

    static void drawRotatedRect(MatOfPoint2f approx, Mat drawOn, String color) {
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

    public double[] getCenter() {
        return center;
    }

    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }
}