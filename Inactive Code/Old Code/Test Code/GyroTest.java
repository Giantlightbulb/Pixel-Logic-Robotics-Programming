package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Gyro Test", group="Sensor Test")


public class GyroTest extends LinearOpMode {
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode(){

        motor1 = hardwareMap.get(DcMotor.class,     "motor1");
        motor2 = hardwareMap.get(DcMotor.class,     "motor2");
        motor3 = hardwareMap.get(DcMotor.class,     "motor3");
        motor4 = hardwareMap.get(DcMotor.class,     "motor4");

        boolean lastResetState = false;
        boolean curResetState  = false;

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class,    "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

    waitForStart();

        telemetry.log().clear();
        telemetry.log().add("Press A & B to reset heading");

    while (opModeIsActive()){

        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        int rawX = modernRoboticsI2cGyro.rawX();
        int rawY = modernRoboticsI2cGyro.rawY();
        int rawZ = modernRoboticsI2cGyro.rawZ();

        //telemetry.log().add("X = " + rawX);
        //telemetry.log().add("Y = " + rawY);
        //telemetry.log().add("Z = " + rawZ);
        telemetry.log().add("Angular Orientation: " + zAngle);
        sleep(100);
        telemetry.clear();
        telemetry.update();

        if (zAngle > 0) {
            motor1.setPower(0.1);
            motor2.setPower(0.1);
            motor3.setPower(0.1);
            motor4.setPower(0.1);
        }else if (zAngle < 0){
            motor1.setPower(-0.1);
            motor2.setPower(-0.1);
            motor3.setPower(-0.1);
            motor4.setPower(-0.1);
        } else if (zAngle == 0) {
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
            telemetry.log().add("Success");



        }





    }
    }
}
