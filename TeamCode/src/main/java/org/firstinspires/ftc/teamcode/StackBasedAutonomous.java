package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="StackBasedAutonomous", group="Autonomous")

public class StackBasedAutonomous extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        //Approximate Speed at full power: 36 inches / 1.75 seconds
        // Left motion motor 2 positive, motor 4 negative
        // Forward motion motor 1 positive motor 3 negative
        //Gyro calibating
        robot.modernRoboticsI2cGyro.calibrate();
        //robot.compass1.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        //robot.compass2.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        robot.compass1.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        robot.compass2.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        while (!isStopRequested() &&
                robot.modernRoboticsI2cGyro.isCalibrating() &&
                robot.compass1.isCalibrating() &&
                robot.compass2.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //Finalized gyro calibration.
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        //Initiate Servos
        robot.latch.setPosition(0.65); // latched
        robot.mascot.setPosition(0.8);//mascot up

        // Start Button
        waitForStart();
        robot.timer.reset();
        toggleLights();
        while(opModeIsActive()) {
            //Gyro
            int rawX = robot.modernRoboticsI2cGyro.rawX();
            int rawY = robot.modernRoboticsI2cGyro.rawY();
            int rawZ = robot.modernRoboticsI2cGyro.rawZ();
            int heading = robot.modernRoboticsI2cGyro.getHeading();
            int integratedZ = robot.modernRoboticsI2cGyro.getIntegratedZValue();

            // Read dimensionalized data from the gyro. This gyro can report angular velocities
            // about all three axes. Additionally, it internally integrates the Z axis to
            // be able to report an absolute angular Z orientation.
            AngularVelocity rates = robot.gyro.getAngularVelocity(AngleUnit.DEGREES);
            float zAngle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Read administrative information from the gyro
            int zAxisOffset = robot.modernRoboticsI2cGyro.getZAxisOffset();
            int zAxisScalingCoefficient = robot.modernRoboticsI2cGyro.getZAxisScalingCoefficient();
            telemetry.addLine()
                    .addData("Gyro Sensor", "");
            telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
            telemetry.addData("angle", "%s deg", formatFloat(zAngle));
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.addData("integrated Z", "%3d", integratedZ);
            telemetry.addLine()
                    .addData("rawX", formatRaw(rawX))
                    .addData("rawY", formatRaw(rawY))
                    .addData("rawZ", formatRaw(rawZ));
            telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
            //Compass 1
            telemetry.addLine()
                    .addData("Compass1:", "");
            telemetry.addLine()
                    .addData("Acceleration:", robot.compass1.getAcceleration())
                    .addData("Heading:", robot.compass1.getDirection());
            //Compass 2
            telemetry.addLine()
                    .addData("Compass2:", "");
            telemetry.addLine()
                    .addData("Acceleration:", robot.compass1.getAcceleration())
                    .addData("Heading:", robot.compass1.getDirection());
            //Range Sensor
            telemetry.addLine()
                    .addData("Range Sensor:", "");
            telemetry.addLine()
                    .addData("CM Optical:", robot.rangeSensor.cmOptical())
                    .addData("CM Ultra Sonic:", robot.rangeSensor.cmUltrasonic());
            //Color Sensor
            telemetry.addLine()
                    .addData("Color Sensor:", "");
            telemetry.addLine()
                    .addData("Color Sensor", "null");
            //Touch Sensor
            telemetry.addLine()
                    .addData("Touch Sensor:", "");
            telemetry.addLine()
                    .addData("Touch Sensor", robot.touch.getValue());
            //ODS
            telemetry.addLine()
                    .addData("ODS:", "");
            telemetry.addLine()
                    .addData("raw light:", robot.ods.getRawLightDetected())
                    .addData("light", robot.ods.getLightDetected());
            telemetry.update();
        }
        telemetry.clear();
    }

    private void toggleLights() {
        robot.ods.enableLed(true);
        if (robot.colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)robot.colorSensor;
            light.enableLight(!light.isLightOn());
        }
        robot.rangeSensor.enableLed(true);
    }
    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }
}
