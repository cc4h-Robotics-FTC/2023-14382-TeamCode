package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwingArm {

    private DcMotor arm1;
    private Servo wrist;
    private Servo hand;
    private double wristPos;
    private double handPos;

    public SwingArm(HardwareMap map) {
        arm1 = map.get(DcMotor.class, "shoulder");
        wrist = map.get(Servo.class, "wrist");
        hand = map.get(Servo.class, "hand");
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void setWrist(double pos) {
        wristPos = wristPos;
        wrist.setPosition(pos);
    }
    public void setHand( double pos) {
        handPos = handPos;
        hand.setPosition(pos);
    }
    public void setPower(double p){

        arm1.setPower(p);
    }

    public void addTelemetry(Telemetry t){
        t.addData("wrist pos", wristPos);
        t.addData("hand pos", handPos);

    }



}
