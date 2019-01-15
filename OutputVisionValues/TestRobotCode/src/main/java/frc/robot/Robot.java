
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class Robot extends TimedRobot {

  /*
  TalonSRX rightTalon, leftTalon;
  VictorSPX right1, right2, left1, left2;
  */

  @Override
  public void robotInit() {

    /*
    rightTalon = new TalonSRX(3);
    leftTalon = new TalonSRX(0);

    right1 = new VictorSPX(4);
    right2 = new VictorSPX(5);
    left1 = new VictorSPX(1);
    left2 = new VictorSPX(2);
    */
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

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");
    //NewworkTableEntry ledMode = table.getEntry("ledMode");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    
    int tshortVal = (int)tshort.getDouble(0.0);
    int tlongVal = (int)tlong.getDouble(0.0);
    
    //ledMode.setNumber(3);

    //post to smart dashboard periodically
    System.out.println("LimelightX " + x);
    System.out.println("LimelightY: " + y);
    System.out.println("LimelightArea: " + area);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    System.out.println("LimelightTShort: " + tshortVal);
    System.out.println("LimelightTLong: " + tlongVal);
    System.out.println("");

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

 
  @Override
  public void testPeriodic() {
  }
}
