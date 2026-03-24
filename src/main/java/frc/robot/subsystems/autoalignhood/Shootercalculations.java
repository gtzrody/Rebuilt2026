package frc.robot.subsystems.autoalignhood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
 

public class Shootercalculations extends SubsystemBase {
 
    private final InterpolatingDoubleTreeMap m_hoodAngleTable = new InterpolatingDoubleTreeMap();
 
    public Shootercalculations() {
        File deployDir = Filesystem.getDeployDirectory();
        File csvFile = new File(deployDir, "shooter_data.csv");
 
        try (BufferedReader br = new BufferedReader(new FileReader(csvFile))) {
            String line;
            while ((line = br.readLine()) != null) {
               
                if (line.isBlank() || line.startsWith("#")) continue;
 
                String[] values = line.split(",");
                if (values.length >= 2) {
                    double distanceMeters    = Double.parseDouble(values[0].trim());
                    double hoodAngleDegrees  = Double.parseDouble(values[1].trim());
                    m_hoodAngleTable.put(distanceMeters, hoodAngleDegrees);
                }
            }
            System.out.println("[ShooterCalculations] shooter_data.csv loaded successfully.");
        } catch (Exception e) {
            System.err.println("[ShooterCalculations] Failed to load shooter_data.csv: " + e.getMessage());
            e.printStackTrace();
        }
    }
 
    /**
     * 
     * 
     *
     * @param distanceMeters Distance from the robot to the hub center, in meters.
     * @return Interpolated hood angle in degrees.
     */
    public double getHoodAngle(double distanceMeters) {
        return m_hoodAngleTable.get(distanceMeters);
    }
}