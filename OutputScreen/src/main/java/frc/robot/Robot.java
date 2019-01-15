
package frc.robot;

import edu.wpi.cscore.VideoSource;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;



public class Robot extends TimedRobot {
  
  @Override
  public void robotInit() {
   
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

  }

  
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tv = table.getEntry("tv");

    Shuffleboard.startRecording();

    NetworkTableEntry myBoolean = Shuffleboard.getTab("LiveWindow")
    .add("Sans", true)
    .withWidget("Toggle Button")
    .getEntry();

    SendableCameraWrapper.wrap(CameraServer.getInstance().putVideo("Cam", 100, 100));

  }
}
