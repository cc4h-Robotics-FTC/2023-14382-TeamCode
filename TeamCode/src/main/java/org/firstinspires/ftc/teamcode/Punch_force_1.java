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
    Arm arm;
    boolean buttonNotPushed = true;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        servo = hardwareMap.get(Servo.class, "Grabber");

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

            if (gamepad2.a) {
                servo.setPosition(0);
            }
            if (gamepad2.b) {
                servo.setPosition(50);
            }

            if (gamepad2.dpad_up && buttonNotPushed) {
                arm.moveElbowByDegrees(90);
                buttonNotPushed = false;
            }
            if (gamepad2.dpad_down && buttonNotPushed) {
                arm.moveShoulderByDegrees(-90);
                buttonNotPushed = false;
            }
//            if (gamepad2.dpad_up) {
//                arm.moveElbowByDegrees(7);
//            }
//            if (gamepad2.dpad_down) {
//                arm.moveElbowByDegrees(-7);
//            }

//            if (gamepad1.a) {
//                arm.moveElbowByDegrees(-45);
//            }
//            if (gamepad1.x) {
//                arm.moveShoulderByDegrees(5);
//            }
//            if (gamepad1.b) {
//                arm.moveShoulderByDegrees(-5);
//            }

//            arm.moveShoulder(gamepad2.left_stick_y);
//            arm.moveElbow(gamepad2.right_stick_y);




            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            arm.addTelementry(telemetry);
            telemetry.update();
        }
    }
}
