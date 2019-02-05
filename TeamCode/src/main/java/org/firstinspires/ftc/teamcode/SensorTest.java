package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorTest extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();

    public void runOpMode() {
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        robot.init(this, hardwareMap, telemetry);
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
        robot.mascot.setPosition(0.8);//mascot up

        // Start Button
        waitForStart();
        robot.timer.reset();
        toggleLights();
        robot.rangeSensor.enableLed(true);
        robot.ods.enableLed(true);
        if (robot.colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)robot.colorSensor;
            light.enableLight(true);
        }
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
                    .addData("Compass1", "");
            telemetry.addLine()
                    .addData("Acceleration", robot.compass1.getAcceleration())
                    .addData("Heading", robot.compass1.getDirection());
            //Compass 2
            telemetry.addLine()
                    .addData("Compass2", "");
            telemetry.addLine()
                    .addData("Acceleration", robot.compass2.getAcceleration())
                    .addData("Heading", robot.compass2.getDirection());
            //Range Sensor
            telemetry.addLine()
                    .addData("Range Sensor:", "");
            telemetry.addLine()
                    .addData("CM Optical", robot.rangeSensor.cmOptical())
                    .addData("CM Ultra Sonic", robot.rangeSensor.cmUltrasonic())
                    .addData("Distance", robot.rangeSensor.getDistance(DistanceUnit.CM));
            //Color Sensor
            telemetry.addLine()
                    .addData("Color Sensor", "");
            NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();

            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", hsvValues[0])
                    .addData("S", hsvValues[1])
                    .addData("V", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", colors.alpha)
                    .addData("r", colors.red)
                    .addData("g", colors.green)
                    .addData("b", colors.blue);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", Color.alpha(color))
                    .addData("r", Color.red(color))
                    .addData("g", Color.green(color))
                    .addData("b", Color.blue(color));

            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", Color.alpha(color))
                    .addData("r", Color.red(color))
                    .addData("g", Color.green(color))
                    .addData("b",  Color.blue(color));

            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);
            //Touch Sensor
            telemetry.addLine()
                    .addData("Touch Sensor", "");
            telemetry.addLine()
                    .addData("Touch", robot.touch.getValue());
            //ODS
            telemetry.addLine()
                    .addData("ODS", "");
            telemetry.addLine()
                    .addData("raw light", robot.ods.getRawLightDetected())
                    .addData("light", robot.ods.getLightDetected());
            telemetry.update();
        }
        telemetry.clear();
        robot.rangeSensor.enableLed(false);
        robot.ods.enableLed(false);
        if (robot.colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)robot.colorSensor;
            light.enableLight(false);
        }
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
