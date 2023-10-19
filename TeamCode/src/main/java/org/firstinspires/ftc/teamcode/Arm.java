package org.firstinspires.ftc.teamcode;

import static java.lang.Math.acos;
import static java.lang.Math.atan;
import static java.lang.Math.cos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    private double PI = Math.PI;
    private DcMotor shoulder;
    private DcMotor elbow;
    private double lowerLength = 15.125;
    private double upperLength = 16;
    private int shoulder_pos;
    private int elbow_pos;
    private int shoulder_angle = 0;
    private int elbow_angle = 0;
    public Arm(HardwareMap map) {
        shoulder = map.get(DcMotor.class, "shoulder");
        elbow = map.get(DcMotor.class, "elbow");

        elbow_pos = elbow.getCurrentPosition();

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_pos = shoulder.getCurrentPosition();

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);


    }



    public void setPosition(double x, double z) {
        // *** Inverse Kinematics ***

        // calculate modification to shoulder angle and arm length

        double shoulderAngle2a = atan(z/x);
        double shoulderAngle2aDegrees = shoulderAngle2a * (180/PI);    // degrees
        double shoulderAngle2 = shoulderAngle2a - 0.7853908;  // take away the default 45' offset (in radians)
        double shoulderAngle2Degrees = shoulderAngle2 * (180/PI);    // degrees
//        shoulderMs2 = shoulderAngle2Degrees * 11;

        double z2 = x/cos(shoulderAngle2a);     // calc new arm length to feed to the next bit of code below

        // ****************

        // calculate arm length based on upper/lower length and elbow and shoulder angle
        double shoulderAngle1a = (sq(upperLength) + sq(z2) - sq(lowerLength)) / (2 * upperLength * z2);
        double shoulderAngle1 = acos(shoulderAngle1a);     // radians
        double elbowAngle = PI - (shoulderAngle1 *2);       // radians

        // calc degrees from angles
        double shoulderAngleDegrees = shoulderAngle1 * (180/PI);    // degrees
        double elbowAngleDegrees = elbowAngle * (180/PI);              // degrees

        moveToAngle(shoulderAngleDegrees, elbowAngleDegrees);

        // *** end of Inverse Kinematics ***

//        double x = 0;
//
//        double b = atan2(y, x) * (180 / 3.1415); // base angle
//
//        double l = sqrt(x * x + y * y); // x and y extension
//
//        double h = sqrt(l * l + z * z);
//
//        double phi = atan(z / l) * (180 / 3.1415);
//
//        double theta = acos((h / 2) / 75) * (180 / 3.1415);
//
//        double a1 = phi + theta; // angle for first part of the arm
//        double a2 = phi - theta; // angle for second part of the arm
//
//        moveToAngle(a1, a2);

    }


    public void powerShoulder(double p) {
        shoulder.setPower(p);
    }

    public void powerElbow(double p) {
        elbow.setPower(p);

    }

    void moveToAngle(double a1, double a2) {

    }

    private double sq(double i) {
        return i * i;
    }

    public void moveShoulderToAngle(int degrees){
        int angle_diff = degrees - shoulder_angle;
        int diff_ticks = getAngleToTicks(angle_diff);
        int target_ticks = shoulder.getCurrentPosition() + diff_ticks;
        shoulder.setTargetPosition(target_ticks);
    }



    public int getAngleToTicks(int i) {
        return Double.valueOf((double)i * 0.8).intValue();
    }
}
