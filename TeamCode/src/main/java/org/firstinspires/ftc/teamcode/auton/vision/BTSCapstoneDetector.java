package org.firstinspires.ftc.teamcode.auton.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BTSCapstoneDetector extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();

    // Scalar low = new Scalar(0, 0, 0, 0);
    // Scalar high = new Scalar(180, 255, 30, 0);
    // Scalar high = new Scalar(0, 0, 49.8, 0);

    // Yellow Ex
    // Scalar low = new Scalar(23, 50, 70);
    // Scalar high = new Scalar(32, 255, 255);
    Scalar low = new Scalar(20, 142, 20);
    Scalar high = new Scalar(255, 163, 90);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(57.5,98);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(145,98);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(232.5,98);
    static final int REGION_WIDTH = 30;
    static final int REGION_HEIGHT = 30;

    static double PERCENT_COLOR_THRESHOLD = 0.6;

    public enum CapstoneLocation {
        LEFT,
        MID,
        RIGHT,
        NOT_FOUND,
    }

    private CapstoneLocation capstoneLocation;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    public BTSCapstoneDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mat, low, high, mat);

        // Blurs the image to smooth it out and reduce unwanted pixels
        Imgproc.GaussianBlur(mat, mat, new Size(11, 15), 0.0);

        Mat left = mat.submat(new Rect(region1_pointA, region1_pointB));
        Mat mid = mat.submat(new Rect(region2_pointA, region2_pointB));
        Mat right = mat.submat(new Rect(region3_pointA, region3_pointB));

        double leftValue = Core.sumElems(left).val[0] / new Rect(region1_pointA, region1_pointB).area() / 255;
        double midValue = Core.sumElems(mid).val[0] / new Rect(region1_pointA, region1_pointB).area() / 255;
        double rightValue = Core.sumElems(mid).val[0] / new Rect(region1_pointA, region1_pointB).area() / 255;

        left.release();
        mid.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Mid raw value", (int) Core.sumElems(mid).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left %", Math.round(leftValue + 100) + "%");
        telemetry.addData("Mid %", Math.round(midValue + 100) + "%");
        telemetry.addData("Right %", Math.round(rightValue + 100) + "%");

        boolean capstoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean capstoneMid = midValue > PERCENT_COLOR_THRESHOLD;
        boolean capstoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if(capstoneLeft && capstoneRight && capstoneMid) {
            // not found
            capstoneLocation = CapstoneLocation.NOT_FOUND;
            telemetry.addData("Capstone location", "not found, defaulting to lowest level");
        } if(capstoneLeft) {
            capstoneLocation = CapstoneLocation.LEFT;
        } if(capstoneRight) {
            // right
            capstoneLocation = CapstoneLocation.RIGHT;
        } if(capstoneMid) {
            // mid
            capstoneLocation = CapstoneLocation.MID;
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorCapstone = new Scalar(0, 255, 0);
        Scalar defaultColor = new Scalar(255, 0, 0);


        Imgproc.rectangle(mat, new Rect(region1_pointA, region1_pointB), capstoneLocation == CapstoneLocation.LEFT ? colorCapstone:defaultColor);
        Imgproc.rectangle(mat, new Rect(region2_pointA, region2_pointB), capstoneLocation == CapstoneLocation.MID ? colorCapstone:defaultColor);
        Imgproc.rectangle(mat, new Rect(region3_pointA, region3_pointB), capstoneLocation == CapstoneLocation.RIGHT ? colorCapstone:defaultColor);

        return mat;
    }

    public CapstoneLocation getLocation() {
        return capstoneLocation;
    }
}
