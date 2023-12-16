package org.firstinspires.ftc.teamcode;

import static org.mockito.Mockito.mock;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.teamcode.fakes.FakeHardwareMap;
import org.firstinspires.ftc.teamcode.fakes.drive.FakeExtendedDcMotor;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class ArmTest {
    private FakeHardwareMap hardwareMap;
    private FakeExtendedDcMotor shoulder;
    private FakeExtendedDcMotor elbow;

    @Test()
    public void testAngleToTicks() {

        Arm a = new Arm(hardwareMap);
        int x = a.getAngleToTicks(90);
        Assert.assertEquals( 72,x);

        x = a.getAngleToTicks(360);
        Assert.assertEquals( 288,x);

    }
    @Test()
    public void testElobwAngleToTicks() {
//
//        Arm a = new Arm(hardwareMap);
//        int x = a.getElbowAngleToTicks(90);
//        Assert.assertEquals( 187,x);
//
//        x = a.getElbowAngleToTicks(360);
//        Assert.assertEquals( 748,x);
//
//        x = a.getElbowAngleToTicks(1);
//        Assert.assertEquals( 2,x);
    }


    @Test
    public void testMoveMotorToAngle() {
//        Arm a = new Arm(hardwareMap);
//        shoulder.setCurrentPosistion(100);
//        a.moveShoulderToAngle(90);
//        int x = a.getAngleToTicks(90);
//        Assert.assertEquals(x+100, shoulder.getCurrentPosition());


    }

    @Before
    public void setup() {
        hardwareMap = new FakeHardwareMap(mock(Context.class), mock(OpModeManagerNotifier.class));

        shoulder = new FakeExtendedDcMotor();
        hardwareMap.addDevice("shoulder", shoulder);

        elbow = new FakeExtendedDcMotor();
        hardwareMap.addDevice("elbow", elbow);
    }
}
