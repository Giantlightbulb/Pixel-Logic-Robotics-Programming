//HARDWARE OMNI

package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Sensors
//   Gyro
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//  ODS
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

//Touch Sensor
import com.qualcomm.robotcore.hardware.TouchSensor;
//  Color Sensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

//Range Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

//Compass Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

//Android App Control
import android.app.Activity;
import android.graphics.Color;
import android.view.View;


public class ArmHardwareOmni {
    /* local OpMode members. */
    private HardwareMap hwMap = null;
    public ElapsedTime timer = new ElapsedTime();

    //Initializes hardware variables
    //Drive train motors
    public DcMotor frontDrive; //
    public DcMotor backDrive; //
    public DcMotor leftDrive; //
    public DcMotor rightDrive; //

    public DcMotor forklift; //Articulating Arm
    //Telescoping lift motor
    public DcMotor verticalLift; //Vertical Extension

    //Chariot Motors
    //Arm
    public DcMotor leftExtension; //Left Arm Extender
    //Back Flip
    public DcMotor rightExtension; //Right Arm Extender

    //Servos
    //Mascot
    public Servo mascot;
    //Bucket Arm
    public Servo latch;
    //Whipper
    public CRServo vacuum;

    //Initializes Sensors
    //  Gyro
    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    //  Touch Sensor
    public TouchSensor touch;

    //Axes
    public AxesReference aRefInt = AxesReference.INTRINSIC;
    public AxesOrder aOrderXYZ = AxesOrder.XYZ;
    public AngleUnit aUnit = AngleUnit.DEGREES;
    NormalizedRGBA colors = new NormalizedRGBA();
    //  ODS
    public OpticalDistanceSensor ods;
    //  Color Sensor
    public NormalizedColorSensor colorSensor;
    //  Compass Sensors
    public ModernRoboticsI2cCompassSensor compass1;
    public ModernRoboticsI2cCompassSensor compass2;

    //  Range Sensor
    public ModernRoboticsI2cRangeSensor rangeSensor;
    //  Initializes layout
    public View relativeLayout;

    //Constructor (Currently empty)
    public ArmHardwareOmni() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        //Hardware definitions
        //Drivetrain motors
        frontDrive = hwMap.get(DcMotor.class, "frontDrive");
        backDrive = hwMap.get(DcMotor.class, "backDrive");
        backDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //Articulating Arm
        forklift = hwMap.get(DcMotor.class, "forklift");
        //Telescoping lift motor
        verticalLift = hwMap.get(DcMotor.class, "verticalLift");
        //Tape Measure Extensions
        leftExtension = hwMap.get(DcMotor.class, "leftExtension");
        rightExtension = hwMap.get(DcMotor.class, "rightExtension");

        //Servos
        //servo1 = Mascot Dump
        mascot = hwMap.get(Servo.class, "mascot");
        //servo2 = Robot Lock
        latch = hwMap.get(Servo.class, "latch");
        //servo3 = Vacuum Whipper
        vacuum = hwMap.get(CRServo.class, "vacuum");

        //  Sensors
        //  Touch Sensor
        touch = hwMap.get(ModernRoboticsTouchSensor.class, "touch");
        //  Gyro
        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        //  ODS
        ods = hwMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        //  Color Sensor
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        //      Detects switchable light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        //  Compasses
        compass1 = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass1");
        compass2 = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass2");
        //  Range Sensor
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //  Application Color
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);

        //  Set all motors to zero power
        frontDrive.setPower(0);
        backDrive.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        forklift.setPower(0);
        verticalLift.setPower(0);
        leftExtension.setPower(0);
        rightExtension.setPower(0);

    }

}
