package org.firstinspires.ftc.teamcode;

import static java.lang.Math.acos;
import static java.lang.Math.atan;
import static java.lang.Math.cos;

import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private double PI = Math.PI;
    private ArmJoint shoulder;
    private ArmJoint elbow;
    private double lowerLength = 15.125;
    private double upperLength = 16;
    public Arm(HardwareMap map) {

       elbow = new ArmJoint(map, "elbow", 2.6, 288.0/360.0);
       shoulder = new ArmJoint(map, "shoulder", 2.7,288.0/360.0 );

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

    void moveToAngle(double a1, double a2) {

    }

    private double sq(double i) {
        return i * i;
    }

    public void moveShoulderByDegrees(int degrees){
        shoulder.moveJointByDegrees(degrees);

    }

    public void moveElbowByDegrees(int degrees){
        elbow.moveJointByDegrees(degrees);

    }
    public void addTelementry(Telemetry telemetry) {
        telemetry.addData("shoulder Running to ticks ",  " %7d", shoulder.getJoint_target_ticks());

        telemetry.addData("shoulder Running to degrees ",  " %7d", shoulder.getJoint_angle());
        telemetry.addData("shoulder Currently at",  " at %7d",
                    shoulder.getCurrentPosition());
        telemetry.addData("shoulder buys? ", shoulder.isBusy());
        telemetry.addData("elbow Running to ticks",  " %7d", elbow.getJoint_target_ticks());

        telemetry.addData("elbow Running to degrees",  " %7d", elbow.getJoint_angle());
        telemetry.addData("elbow Currently at",  " at %7d",
                elbow.getCurrentPosition());
        telemetry.addData("Elbow buys? ", elbow.isBusy());

    }

    public int getAngleToTicks(int i) {
        return Double.valueOf((double)i * 0.8).intValue();
    }

    public void moveShoulderToAngle(int i) {

    }
}
