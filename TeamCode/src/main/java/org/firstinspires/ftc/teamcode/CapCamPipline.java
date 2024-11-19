package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Math.abs;
import static java.lang.Math.min;

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
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@Config
public class CapCamPipline extends OpenCvPipeline {
    public static int retVal = 0;
    List<MatOfPoint> contours = new ArrayList<>();
    public static double RUH = 10, RLH = 160, RS = 100, RV = 100, BH = 100, BUH = 120, BS = 150, BV = 30, YH = 18, YUH = 30, YS = 110, YV = 180;
    Mat hsv = new Mat();
    Mat blurredImage = new Mat();
    Mat mask = new Mat(), mask2 = new Mat(), mask3 = new Mat(), mask4 = new Mat();
    Mat colorMask = new Mat(), allMask = new Mat();
    Mat hierarchy = new Mat();
    Mat boundingImage = new Mat(), firstBoundingImage = new Mat();

    public static double AREA_THRESH = .85, FCL = 1, EPSILON = 0.04, UP_TOLERANCE = 2, DOWN_TOLERANCE =1.15, CLASSUP_TOL = 0.8, CLASSDOWN_TOL = 0.7;
    double objectWidth = 3.5;  // Replace with your object's width in real-world units (e.g., centimeters)
    double objectHeight = 1.5;  // Replace with your object's height in real-world units

    // Define the 3D coordinates of the object corners in the object coordinate space
    MatOfPoint2f imagePoints = new MatOfPoint2f(), contour2f = new MatOfPoint2f(), approx = new MatOfPoint2f();
    MatOfPoint approxMat;
    MatOfPoint3f objectPoints = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));

    Point[] orderedRectPoints;
    Mat rvec = new Mat();
    Mat tvec = new Mat();


    static final int CONTOUR_LINE_THICKNESS = 2;

    double[] center = {0, 0, 0};

    /*
     * Camera Calibration Parameters
     */
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();
    RotatedRect minAreaRect;

    int color =0;

    public CapCamPipline() {
        double fx = 822.317f; // Replace with your camera's focal length in pixels
        double fy = 822.317f;
        double cx = 319.495f; // Replace with your camera's principal point x-coordinate (usually image width / 2)
        double cy = 242.502f; // Replace with your camera's principal point y-coordinate (usually image height / 2)
        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);
        distCoeffs = new MatOfDouble(-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0);
    }


    public void resetCenter() {
        center = new double[]{0, 0, 0};
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar rlFilt = new Scalar(RLH, RS, RV),
                ruFilt = new Scalar(180, 255, 255),
                rllFilt = new Scalar(0,RS,RV),
                rulFilt = new Scalar(RUH, 255,255),
                blFilt = new Scalar(BH, BS, BV),
                buFilt = new Scalar(BUH, 255, 255),
                ylFilt = new Scalar(YH, YS, YV),
                yuFilt = new Scalar(YUH, 255, 255);
        mask = new Mat();
        mask2 = new Mat();
        mask3 = new Mat();
        mask4 = new Mat();
        colorMask = new Mat();
        allMask = new Mat();
        Core.inRange(hsv, rlFilt, ruFilt, mask);
        Core.inRange(hsv, rllFilt, rulFilt, mask2);
        Core.inRange(hsv, blFilt, buFilt, mask3);
        Core.inRange(hsv, ylFilt, yuFilt, mask4);
        Core.bitwise_or(mask,mask2,mask);
        if(color ==0)
            colorMask = mask;
        else if(color == 1)
            colorMask = mask3;
        else
            colorMask = mask4;
        Core.bitwise_or(mask3,mask4,allMask);
        Core.bitwise_or(allMask,mask, allMask);
        contours = new ArrayList<>();
        Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        boundingImage = input.clone();
        ArrayList<Double[]> colorCoords = contoursToCoords();
        firstBoundingImage = boundingImage.clone();
        boundingImage = input.clone();
        contours = new ArrayList<>();
        Imgproc.findContours(allMask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<Double[]> allCoords = contoursToCoords();
        Double[] centerd = matchedCoords(colorCoords, allCoords);
        if(centerd[0] != 100) center = convertToDoubleArray(centerd);
        if (retVal == 0) {
            return firstBoundingImage;
        } else if (retVal == 1) {
            return boundingImage;
        } else if (retVal == 2) {
            return mask;
        } else if (retVal == 3) {
            return mask3;
        } else if(retVal == 4){
            return mask4;
        }
        else if(retVal == 5){
            return colorMask;
        }
        else if(retVal == 6){
            return allMask;
        }
        return boundingImage;
    }
    double[] convertToDoubleArray(Double[] wrapperArray) {
        double[] primitiveArray = new double[wrapperArray.length];

        for (int i = 0; i < wrapperArray.length; i++) {
            primitiveArray[i] = wrapperArray[i]; // Auto-unboxing
        }

        return primitiveArray;
    }
    public Double[] matchedCoords(ArrayList<Double[]> colorCoords, ArrayList<Double[]> allCoords){
        ArrayList<Double[]> matchedCenters = new ArrayList<>();
        double minDist = 1000;
        int coord = 0;
        for(int i = 0 ; i < colorCoords.size(); i++){
            for(int j=0;j<allCoords.size();j++){
                double x = colorCoords.get(i)[0] - allCoords.get(j)[0];
                double y = colorCoords.get(i)[1] - allCoords.get(j)[1];
                if(x*x+y*y<9){
                    matchedCenters.add(colorCoords.get(i));
                    if(colorCoords.get(i)[0]*colorCoords.get(i)[0]+colorCoords.get(i)[1]*colorCoords.get(i)[1]<minDist){
                        coord = matchedCenters.size()-1;
                        minDist = colorCoords.get(i)[0]*colorCoords.get(i)[0]+colorCoords.get(i)[1]*colorCoords.get(i)[1];
                    }
                }
            }
        }
        if(matchedCenters.isEmpty())
            return new Double[] {100.0,100.0,100.0};
        return matchedCenters.get(coord);
    }

    public ArrayList<Double[]> contoursToCoords(){
        ArrayList<Double[]> centers = new ArrayList<>();
        // Set acceptable aspect ratio range
        double minAspectRatio = 1.5 - DOWN_TOLERANCE;
        double maxAspectRatio = 1.5 + UP_TOLERANCE;
        double minAreaThreshold = 10000;  // Minimum area threshold

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

            if (minAreaRect.size.width != 0 && minAreaRect.size.height != 0 && Imgproc.contourArea(contour) / (minAreaRect.size.height * minAreaRect.size.width) > AREA_THRESH) {
                Point[] box = new Point[4];
                box = approxMat.toArray();
                Point[] orded = orderPoints(box);
                double[] distances = {distance(orded[0], orded[1]), distance(orded[1], orded[2]), distance(orded[0], orded[2])};
                Arrays.sort(distances);
                double width = distances[1];
                double height = distances[0];
                if (height != 0) {  // Avoid division by zero
                    double aspectRatio = width / height;
                    if (minAspectRatio <= aspectRatio && aspectRatio <= maxAspectRatio) {
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

                        MatOfPoint3f rlObjectPoints = new MatOfPoint3f();
                        rlObjectPoints = objectPoints;
                        orderedRectPoints = orderCorner(orderedRectPoints);
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
                            double consta = 1;
                            centers.add(new Double[]{coords[0] * consta,coords[1] * consta,coords[2] * consta});
                            packet.put("CAM X", center[0]);
                            packet.put("CAM y", center[1]);
                            packet.put("CAM Z", center[2]);
                            packet.put("aspectRatio", aspectRatio);
                            packet.put("area", Imgproc.contourArea(contour));
                            packet.put("color", color);

                        }
                    }
                }
            }
        }
        return centers;
    }


    private static double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }

    public static Point[] orderCorner(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }
        Point[] sortedByDistance = {pts[0], null, null};
        double d1 = distance(pts[0], pts[1]);
        double d2 = distance(pts[0], pts[2]);
        double d3 = distance(pts[0], pts[3]);
        Point[] orderedPts = new Point[4];
        if (d1 < d2 && d1 < d3) {
            //  3 0 1 2, 3 1 0 2, 2 1 0 3, 2 0 1 3
            int[] shorts = {0, 1};
            int[] longs = {2, 3};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        } else if (d2 < d3) {
            int[] shorts = {0, 2};
            int[] longs = {1, 3};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        } else {
            int[] shorts = {0, 3};
            int[] longs = {1, 2};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        }


        return orderedPts;
    }

    // Helper method to calculate the dot product of two vectors
    double dotProduct(Point p1, Point p2) {
        return p1.x * p2.x + p1.y * p2.y;
    }

    // Helper method to calculate the magnitude of a vector
    double magnitude(Point p) {
        return Math.sqrt(p.x * p.x + p.y * p.y);
    }

    // Helper method to calculate the angle between two vectors represented by Points
    double calculateAngle(Point p1, Point p2) {
        double dotProd = dotProduct(p1, p2);
        double magP1 = magnitude(p1);
        double magP2 = magnitude(p2);
        double cosTheta = dotProd / (magP1 * magP2);
        return Math.acos(cosTheta) * (180.0 / Math.PI); // Return the angle in degrees
    }
    public static Point[] orderPoints(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }

        // Calculate the center of the frame
        double centerX = 640;
        double centerY = 360;
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
        if (isCC(thirdPoint, orderedPts[0], orderedPts[1], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[0], orderedPts[1], fourthPoint};
        } else if (isCC(thirdPoint, orderedPts[1], orderedPts[0], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[1], orderedPts[0], fourthPoint};

        } else if (isCC(fourthPoint, orderedPts[1], orderedPts[0], thirdPoint)) {
            orderedPts = new Point[]{fourthPoint, orderedPts[1], orderedPts[0], thirdPoint};

        } else {
            orderedPts = new Point[]{fourthPoint, orderedPts[0], orderedPts[1], thirdPoint};

        }

        return orderedPts;
    }


    public static boolean isCC(Point p1, Point p2, Point p3, Point p4) {
        return isCCW(p1, p2, p3) && isCCW(p1, p3, p4) &&
                isCCW(p2, p3, p4) && isCCW(p1, p2, p4);
    }

    private static boolean isCCW(Point a, Point b, Point c) {
        // Calculate the cross product of vector AB and AC
        double crossProduct = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return crossProduct > 0; // Returns true if the points are in counter-clockwise order
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

    public double[] getCenter() {
        return center;
    }

    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return new Scalar(0,0,255);
            case "Yellow":
                return new Scalar(255,255,0);
            default:
                return new Scalar(255,0,0);
        }
    }

    public void setColor(int color){
        this.color = color;
    }
    public int getColor(){
        return color;
    }
}