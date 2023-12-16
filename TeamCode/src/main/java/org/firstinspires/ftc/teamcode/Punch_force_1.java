package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive")
public class Punch_force_1 extends LinearOpMode {

    Servo servo;
    SwingArm arm;
    boolean buttonNotPushed = true;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = new SwingArm(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo = hardwareMap.get(Servo.class, "plane");

        servo.scaleRange(0.54, 1.0);
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();


            double sPos = gamepad1.left_trigger;
            servo.setPosition(sPos);
            
            double armP = gamepad2.left_stick_y / 2.0;
            arm.setPower(armP);
            double handpos = gamepad2.right_trigger;
            arm.setHand(handpos);
            double wristpos = gamepad2.left_trigger;
            arm.setWrist(wristpos);


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("plane",sPos );
            telemetry.addData("arm power", armP);
            arm.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
