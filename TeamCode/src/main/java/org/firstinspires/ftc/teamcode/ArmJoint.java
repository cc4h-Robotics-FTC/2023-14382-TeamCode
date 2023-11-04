package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmJoint {
    private DcMotor joint;
    private int joint_pos;
    private int joint_angle = 0;
    public int joint_target_ticks = 0;
    private double gearRatio = 0.0;
    private double ticksPerDegree = 0.0;

    public ArmJoint(HardwareMap map, String deviceName, double gearRatio, double ticksPerDegree) {
        this.gearRatio = gearRatio;
        this.ticksPerDegree = ticksPerDegree;
        joint = map.get(DcMotor .class, deviceName);
        joint_pos = joint.getCurrentPosition();
        joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveJointByDegrees(int degrees) {

        int diff_ticks = getJointAngleToTicks(degrees);
        joint_target_ticks = joint.getCurrentPosition() + diff_ticks;
        joint.setTargetPosition(joint_target_ticks);
        joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint.setPower(1);
        joint_angle = joint_angle + degrees;
    }

    public int getJointAngleToTicks(int i) {
        return Double.valueOf((double)i * gearRatio * ticksPerDegree).intValue();
    }

    public int getCurrentPosition() {
        return joint.getCurrentPosition();
    }
    public boolean isBusy() {
        return joint.isBusy();
    }

    public int getJoint_target_ticks(){
        return joint_target_ticks;

    }
    public int getJoint_angle(){
        return joint_angle;
    }

}
