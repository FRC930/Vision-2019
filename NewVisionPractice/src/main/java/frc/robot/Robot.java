package frc.robot;

//----Imports----\\
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Robot extends TimedRobot {

  //----Object Declarations----\\

  // xbox controller used for controlling the robot
  Joystick stick = new Joystick(0);

  // NetworkTable object declaration used to get values from the limelight network table
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // Spark motor controllers
  CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftFollow1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftFollow2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightFollow1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rightFollow2 = new CANSparkMax(6, MotorType.kBrushless);

  //----Constants----\\
  private final double DEFAULT_HORIZONTAL_SPEED = -0.01;
  private final double HORIZONTAL_ANGLE_THRESHOLD = 0.3;

  // default value that the limelight will return if no targets are found
  private final double DEFAULT_LIMELIGHT_VALUE = 0.1234;

  // Controller constants
  private final int A_BUTTON = 1;
  private final int LEFT_Y_AXIS = 1;
  private final int RIGHT_X_AXIS = 4;
  private final double JOYSTICK_DEADBAND = 0.000124;

  private final int CAMERA1_WIDTH = 320;
  private final int CAMERA1_HEIGHT = 240;
  private final int CAMERA1_FPS = 30;
  private final int CAMERA2_WIDTH = 160;
  private final int CAMERA2_HEIGHT = 120;
  private final int CAMERA2_FPS = 30;

  //----Variable Declarations----\\

  // get the angle of the horizontal offset between the camera's crosshair and the target's crosshair
  NetworkTableEntry tx = limelightTable.getEntry("tx");

  // typecast the NetworkTable data into a double
  // getDouble() will return a default value if no value is found
  double horizontalAngle = tx.getDouble(DEFAULT_LIMELIGHT_VALUE);

  // left and right speeds for the drivetrain 
  double leftMovement = 0.0;
  double rightMovement = 0.0;

  // speed of the robot in the forward and backward directions
  double distanceSpeed = 0;

  // speed of the robot in the left and right directions
  double horizontalSpeed = 0;

  // values of the left joystick's x-axis and the right joystick's y-axis
  double stickX = 0;
  double stickY = 0;

  @Override
  public void robotInit() {

    /* set the limelight and USB camera to picture-in-picture mode,
       which means the limelight's camera feed is shown in the
       bottom right corner of the USB camera's feed
    */
    limelightTable.getEntry("stream").setNumber(2);

    // configure 2 motors on each side of the robot to follow the main motor on each side
    leftFollow1.follow(leftMotor);
    rightFollow1.follow(rightMotor);
    leftFollow2.follow(leftMotor);
    rightFollow2.follow(rightMotor);

    // begin sending data to the shuffleboard
    Shuffleboard.startRecording();

    // initialize USB camera objects and begin sending their video streams to shuffleboard
    startCapture();
  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {

    // reset the horizontal and distance speeds every time the code runs
    // this will prevent previous leftover values from moving the motors
    horizontalSpeed = 0;
    distanceSpeed = 0;

    // checks if the A button is currently being pressed, returns a boolean
	  if(stick.getRawButton(A_BUTTON)) {

      /* if the A button is currently pressed, turn on the limelight's LEDs,
         and if the A button is not pressed, turn off the LEDs

         getEntry("ledMode") is used to get the LedMode property from the NetworkTable
         setNumber() is used to set the state of the LedMode property (the integer 1 sets the LEDs off, 3 sets the LEDs on)
         
         ?: is a terenary operator in Java, it is basically a one-line if-else statement
         To use it, give it a boolean variable, the do ?. Then after that, you can use it as
         an if-else statement. 

         stick.getRawButton(A_BUTTON) ? 1 : 0
         In this example, we use stick.getRawButton(A_BUTTON) as our boolean variable
         if stick.getRawButton(A_BUTTON) is down (value is true), then it will return 1
         if stick.getRawButton(A_BUTTON) is up (value is false), then it will return 0

         With the 0 and 1 then, we use it to do some math. 
         When the stick.getRawButton(A_BUTTON) is down (true), it returns 1
         1 + (2 * 1) = 3. 3 is sent into .setNumber(), which turns the LEDs on
      */
      limelightTable.getEntry("ledMode").setNumber(1 + (2 * (stick.getRawButton(A_BUTTON) ? 1 : 0)));

      // rotate the robot towards the target if horizontal angle is greater than the horizontal angle threshold on either side of the target
      rotate(horizontalAngle);
    }

    // Cubing values to create smoother function
    stickX = -Math.pow(stick.getRawAxis(LEFT_Y_AXIS), 3);
    stickY = Math.pow(stick.getRawAxis(RIGHT_X_AXIS), 3);

    // manual drive controls
    if(Math.abs(stickX) > JOYSTICK_DEADBAND) {
      distanceSpeed = stickX;
    }
    if(Math.abs(stickY) > JOYSTICK_DEADBAND) {
      horizontalSpeed = stickY;
    }

    // left and right speeds of the drivetrain
    leftMovement = distanceSpeed + horizontalSpeed;
    rightMovement = distanceSpeed - horizontalSpeed;

    // run the robot based on the left and right speeds of the drive train
    runAt(-leftMovement, rightMovement);
  }

  @Override
  public void testPeriodic() {

  }

   // drive the robot
   private void runAt(double leftSpeed, double rightSpeed) {      
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
   }

   // rotate the robot based on horizontal angle offset between the camera's crosshair and the target's crosshair
   private void rotate(double xAngle) {
      if(Math.abs(xAngle) > HORIZONTAL_ANGLE_THRESHOLD) 
        horizontalSpeed = DEFAULT_HORIZONTAL_SPEED * (-xAngle / 1.5);
   }

   // begin capturing video from two USB cameras and send their video streams to shuffleboard
   private void startCapture() {
    // creates a thread which runs concurrently with the program
		new Thread(() -> {
      // Instantiate the USB cameras and begin capturing their video streams
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
      UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);

      // set the cameras' reolutions and FPS
			camera.setResolution(CAMERA1_WIDTH, CAMERA1_HEIGHT);
      camera.setFPS(CAMERA1_FPS);
			camera2.setResolution(CAMERA2_WIDTH, CAMERA2_HEIGHT);
      camera2.setFPS(CAMERA2_FPS);
		}).start();
	}
}