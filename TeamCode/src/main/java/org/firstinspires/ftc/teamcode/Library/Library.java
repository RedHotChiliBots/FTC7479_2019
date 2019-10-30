package org.firstinspires.ftc.teamcode.Library;

import java.util.Arrays;
import java.util.List;


public class Library {

    public double clip(double val, double max, double min) {
        if (val > max) {
            val = max;
        } else if (val < min) {
            val = min;
        }
        return val;
    }

    /**
     * calcCorrection takes relative position information from the Vuforia image
     * tracking and returns a left and right motor command to track the target.
     * The Vuforia responses are distances measured in inches and available via
     * these methods:
     *    getPosDist (x) - positive is cartesian distance from image
     *    getPosOffset (y) - negative is left, positive is right cartesian distance from image
     *    getPosHeight (z) - negative is down, positive is up cartesian distance from image
     *    getPosLOS (calc) - positive is Line Of Sight distance from image
     *    getPosAngle (calc) - negative is left, positive is right degrees from image center
     *    getRoll - n/a
     *    getPitch - n/a
     *    getYaw - negative is left, positive is right degrees of image in camera frame
     *
     * @param offset
     * @param angle
     * @param yaw
     * @return
     */
    public List<Double> calcCorrection(double offset, double angle, double yaw) {
        double correct = 0.0;
        double left;
        double right;
        double speed = 0.5;

        left = speed;
        right = speed;

//        if ((Math.abs(offset) > 2.0) || (Math.abs(angle) > 2.0)) {
            correct = yaw + (angle / 2);
            correct = clip(correct, 45.0, -45.0);

            left = speed + ((correct / 90.0) * speed);
            right = speed - ((correct / 90.0) * speed);

            left = clip(left, 1.0, -1.0);
            right = clip(right, 1.0, -1.0);
//        }

        return Arrays.asList(left,right);
    }
}

