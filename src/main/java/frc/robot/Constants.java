// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Robot Mode
    public enum RobotMode {
        REAL, // Physical robot hardware
        SIM, // Simulation mode
        REPLAY // Replay mode for log analysis
    }
    public static final RobotMode simMode = RobotMode.SIM;
    public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;
    
    // Battery Voltage Warnings
    public static final double BATTERY_VOLTAGE_CRITICAL = 11.5; // Volts TODO: Adjust later
    public static final double BATTERY_VOLTAGE_WARNING = 12.0; // Volts TODO: Adjust later

    public static final class Controller{
        // Controller Ports
        public static final int DRIVE_CONTROLLER_PORT = 0; // TODO: Set port
        public static final int MANIP_CONTROLLER_PORT = 1; // TODO: Set port

        // Joystick Deadband
        public static final double RIGHT_Y_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static final class Swerve {
        public static final double ROBOT_MASS = Units.lbsToKilograms(100.000); // kg Adjusted value
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS); // TODO: Adjust later
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag TDO: Adjust later

        // Speeds
        public static final double MAX_SPEED = Units.feetToMeters(16.6); // m/s TODO: Adjust later
        public static final double MAX_ACCELERATION = Units.feetToMeters(34.44882); // m/s^2 TODO: Adjust later
        public static final Rotation2d MAX_ROTATION_SPEED = Rotation2d.fromRadians(10); // m/s TODO: Adjust later

        // Auto Speeds
        public static final double AUTO_MAX_SPEED = Units.feetToMeters(.5); // m/s TODO: Adjust later
        public static final double AUTO_MAX_ACCELERATION = Units.feetToMeters(.7); // m/s^2 TODO: Adjust later
        public static final Rotation2d AUTO_MAX_ROTATION_SPEED = Rotation2d.fromRadians(1); // m/s TODO: Adjust later
    }

    public static final class AlgaeIntake{
      private AlgaeIntake() {}; 
      // todo: find id's
      public static final int ROLLER_ID = 11; 
      public static final int PIVOT_ID = 12; 
      public static final int IR_SENSOR_ID = 0; 
      // todo: find p, i, d, g
      public static final double kP = 0; 
      public static final double kI = 0; 
      public static final double kD = 0; 
      public static final double kG = 0;
      // todo: find tolerances
      public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0); 
      public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(0); 
      public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(0); 
      // todo: find rotations
      public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(0);
    }

  public static final class FloorIntake{
    private FloorIntake() {}; 
    //todo: set ports
    public static final int ROLLERMOTOR_PORT = 13; 
    public static final int PIVOTMOTOR_PORT = 14;
    public static final int IR_SENSOR_PORT = 15; 
    public static final int ENCODER_PORT = 16; 

    //todo: find p, i, d, g
    public static final double PIVOT_P = 0; 
    public static final double PIVOT_I = 0; 
    public static final double PIVOT_D = 0;
    public static final double PIVOT_G = 0; 

    //todo: set tolerance
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(69); 
    public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(420); 
    public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(420); 
    
    //todo: set offset in range from 0 to 1
    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(0);
  } 

  public static final class LED{
    private LED() {}; 
    //todo: set ports and length
    public static final int LED_PORT = 17;
    public static final int LED_LENGTH = 10; 
  }

  public static final class Elevator{
    private Elevator() {}; 
    // todo: find id's
    public static final int LEFT_MOTOR_ID = 0; 
    public static final int RIGHT_MOTOR_ID = 0;  
    // todo: find limit switch/servo ports 
    public static final int LIMIT_SWITCH_PORT = 0; 
    public static final int SERVO_PORT = 0;
    // todo: find p, i, d, f values 
    public static final double kP = 0; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.0; 

    public static final double TOLERANCE = 0; // todo: find tolerance
    public static final double CONVERSION_FACTOR = 0.0095758; // todo; check number, converts to meters 
    public static final Rotation2d LOCK_ANGLE = Rotation2d.fromDegrees(69); // todo: find optimal lock angle
    public static final Rotation2d UNLOCK_ANGLE = Rotation2d.fromDegrees(420); // todo: find optimal unlocked angle

    public enum Position{ 
      //todo: set height setpoints in meters
      L1(0), L2(0), L3(0), CLIMB_UP(0), CLIMB_DOWN(0), MIN(0), MAX(0); 
      private final double height;

      Position(double height){
        this.height = height;
      }

      public double getHeight(){
        return height;
      }
    }
  }

    public static final class EndEffector{
      private EndEffector() {}; 
      // todo: set ports 
      public static final int ROLLER_MOTOR_ID = 18; 
      // todo: set channels
      public static final int FRONT_SENSOR_CHANNEL = 19; 
      public static final int BACK_SENSOR_CHANNEL = 20; 
    }
    public static final class Vision {
        // Maximum allowed ambiguity for the cameras
        public static final double MAXIMUM_AMBIGUITY = 0.25; // TODO: Adjust later

        // Camera Configs
        // TODO: Set real values
        public enum Camera {
            ArducamOne(
                "Arducam_1",
                new Rotation3d(Math.toRadians(0), Math.toRadians(-18), Math.toRadians(40)),
                new Translation3d(
                    // Units.inchesToMeters(11.233), 
                    // Units.inchesToMeters(9.691),
                    // Units.inchesToMeters(8.513920)
                    Units.inchesToMeters(9.691),
                    Units.inchesToMeters(-11.233),
                    Units.inchesToMeters(8.513920)
                ),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1)
            ),

            ArducamTwo(
                "Arducam_2",
                new Rotation3d(Math.toRadians(0), Math.toRadians(-18), Math.toRadians(-40)),
                new Translation3d(
                    // Units.inchesToMeters(-11.233), 
                    // Units.inchesToMeters(9.691), 
                    // Units.inchesToMeters(8.513920)
                    Units.inchesToMeters(9.691),
                    Units.inchesToMeters(11.233),
                    Units.inchesToMeters(8.513920)
                ),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1)
            ),

            ArducamThree(
                "Arducam_3",
                new Rotation3d(0, Units.degreesToRadians(-145), 0),
                new Translation3d(
                    Units.inchesToMeters(-4.628),
                    Units.inchesToMeters(-10.687),
                    Units.inchesToMeters(16.129)
                ),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1)
            );


            public final String name;
            public final Rotation3d rotation;
            public final Translation3d translation;
            public final Matrix<N3, N1> singleTagStdDevs;
            public final Matrix<N3, N1> multiTagStdDevs;

            Camera(
            String name,
            Rotation3d rotation,
            Translation3d translation,
            Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs) {
                this.name = name;
                this.rotation = rotation;
                this.translation = translation;
                this.singleTagStdDevs = singleTagStdDevs;
                this.multiTagStdDevs = multiTagStdDevs;
            }
        }
        public static final Pose3d[] CAMERA_POSITIONS = {
            new Pose3d(Camera.ArducamOne.translation, Camera.ArducamOne.rotation),
            new Pose3d(Camera.ArducamTwo.translation, Camera.ArducamTwo.rotation),
            new Pose3d(Camera.ArducamThree.translation, Camera.ArducamThree.rotation),
        };
    }
public static final class VisionConstants{
    public static final double[][] noteDetectionLookupTable = {
        {154.27134704589844, 171.6876983642578, Units.inchesToMeters(0), Units.inchesToMeters(7.5)},
        {178.7086181640625, 170.32672119140625, Units.inchesToMeters(4), Units.inchesToMeters(7.5)},
        {205.19854736328125, 167.13938903808594, Units.inchesToMeters(8), Units.inchesToMeters(7.5)},
        {228.66574096679688, 161.6444854736328, Units.inchesToMeters(12), Units.inchesToMeters(7.5)},
        {246.21170043945312, 157.2861328125, Units.inchesToMeters(16), Units.inchesToMeters(7.5)},
        {259.782958984375, 150.37551879882812, Units.inchesToMeters(20), Units.inchesToMeters(7.5)},
        {272.013427734375, 146.05697631835938, Units.inchesToMeters(24), Units.inchesToMeters(7.5)},
        {155.65867614746094, 137.33753967285156, Units.inchesToMeters(0), Units.inchesToMeters(13.5)},
        {179.44638061523438, 137.10073852539062, Units.inchesToMeters(4), Units.inchesToMeters(13.5)},
        {202.794921875, 136.00408935546875, Units.inchesToMeters(8), Units.inchesToMeters(13.5)},
        {221.5397491455078, 133.6504364013672, Units.inchesToMeters(12), Units.inchesToMeters(13.5)},
        {
          238.55738830566406, 131.46266174316406, Units.inchesToMeters(16), Units.inchesToMeters(13.5)
        },
        {
          251.65420532226562, 128.71676635742188, Units.inchesToMeters(20), Units.inchesToMeters(13.5)
        },
        {262.49658203125, 126.91678619384766, Units.inchesToMeters(24), Units.inchesToMeters(13.5)},
        {155.79156494140625, 110.67668914794922, Units.inchesToMeters(0), Units.inchesToMeters(19.5)},
        {176.4880828857422, 111.09066009521484, Units.inchesToMeters(4), Units.inchesToMeters(19.5)},
        {197.07095336914062, 111.10057067871094, Units.inchesToMeters(8), Units.inchesToMeters(19.5)},
        {213.2230682373047, 111.08323669433594, Units.inchesToMeters(12), Units.inchesToMeters(19.5)},
        {227.31912231445312, 110.8406982421875, Units.inchesToMeters(16), Units.inchesToMeters(19.5)},
        {
          241.66949462890625, 110.21771240234375, Units.inchesToMeters(20), Units.inchesToMeters(19.5)
        },
        {252.04759216308594, 109.3546142578125, Units.inchesToMeters(24), Units.inchesToMeters(19.5)},
        {156.43348693847656, 92.26092529296875, Units.inchesToMeters(0), Units.inchesToMeters(25.5)},
        {173.71957397460938, 92.69190216064453, Units.inchesToMeters(4), Units.inchesToMeters(25.5)},
        {191.1551513671875, 93.2912826538086, Units.inchesToMeters(8), Units.inchesToMeters(25.5)},
        {205.46400451660156, 93.71550750732422, Units.inchesToMeters(12), Units.inchesToMeters(25.5)},
        {220.07833862304688, 94.37662506103516, Units.inchesToMeters(16), Units.inchesToMeters(25.5)},
        {232.0174560546875, 94.9604263305664, Units.inchesToMeters(20), Units.inchesToMeters(25.5)},
        {242.53091430664062, 95.50878143310547, Units.inchesToMeters(24), Units.inchesToMeters(25.5)},
        {156.4423828125, 82.93617248535156, Units.inchesToMeters(0), Units.inchesToMeters(29.5)},
        {172.02281188964844, 83.42843627929688, Units.inchesToMeters(4), Units.inchesToMeters(29.5)},
        {187.8604736328125, 84.26107788085938, Units.inchesToMeters(8), Units.inchesToMeters(29.5)},
        {201.43931579589844, 85.0326156616211, Units.inchesToMeters(12), Units.inchesToMeters(29.5)},
        {214.3050537109375, 85.8529281616211, Units.inchesToMeters(16), Units.inchesToMeters(29.5)},
        {226.25885009765625, 86.8006820678711, Units.inchesToMeters(20), Units.inchesToMeters(29.5)},
        {236.1659698486328, 87.51233673095703, Units.inchesToMeters(24), Units.inchesToMeters(29.5)},
        {156.423828125, 75.72048950195312, Units.inchesToMeters(0), Units.inchesToMeters(33.5)},
        {170.40736389160156, 76.27440643310547, Units.inchesToMeters(4), Units.inchesToMeters(33.5)},
        {184.97666931152344, 77.00422668457031, Units.inchesToMeters(8), Units.inchesToMeters(33.5)},
        {197.5010223388672, 77.55702209472656, Units.inchesToMeters(12), Units.inchesToMeters(33.5)},
        {209.526611328125, 78.64038848876953, Units.inchesToMeters(16), Units.inchesToMeters(33.5)},
        {220.35401916503906, 79.69290161132812, Units.inchesToMeters(20), Units.inchesToMeters(33.5)},
        {230.5920867919922, 80.84618377685547, Units.inchesToMeters(24), Units.inchesToMeters(33.5)},
        {156.3590087890625, 69.85771179199219, Units.inchesToMeters(0), Units.inchesToMeters(37.5)},
        {169.06935119628906, 70.47810363769531, Units.inchesToMeters(4), Units.inchesToMeters(37.5)},
        {182.52525329589844, 71.03211212158203, Units.inchesToMeters(8), Units.inchesToMeters(37.5)},
        {194.46484375, 72.02528381347656, Units.inchesToMeters(12), Units.inchesToMeters(37.5)},
        {205.57937622070312, 72.911865234375, Units.inchesToMeters(16), Units.inchesToMeters(37.5)},
        {215.69342041015625, 73.98934936523438, Units.inchesToMeters(20), Units.inchesToMeters(37.5)},
        {225.42604064941406, 75.15672302246094, Units.inchesToMeters(24), Units.inchesToMeters(37.5)},
        {156.41311645507812, 64.90918731689453, Units.inchesToMeters(0), Units.inchesToMeters(41.5)},
        {168.29574584960938, 65.42889404296875, Units.inchesToMeters(4), Units.inchesToMeters(41.5)},
        {180.56785583496094, 66.30609893798828, Units.inchesToMeters(8), Units.inchesToMeters(41.5)},
        {191.53038024902344, 66.91743469238281, Units.inchesToMeters(12), Units.inchesToMeters(41.5)},
        {202.05491638183594, 68.10053253173828, Units.inchesToMeters(16), Units.inchesToMeters(41.5)},
        {212.24427795410156, 69.29578399658203, Units.inchesToMeters(20), Units.inchesToMeters(41.5)},
        {221.28753662109375, 70.37397766113281, Units.inchesToMeters(24), Units.inchesToMeters(41.5)},
        {156.75836181640625, 60.81660842895508, Units.inchesToMeters(0), Units.inchesToMeters(45.5)},
        {167.243896484375, 61.163150787353516, Units.inchesToMeters(4), Units.inchesToMeters(45.5)},
        {178.677734375, 61.95714569091797, Units.inchesToMeters(8), Units.inchesToMeters(45.5)},
        {188.78147888183594, 62.63084411621094, Units.inchesToMeters(12), Units.inchesToMeters(45.5)},
        {198.72476196289062, 63.67478561401367, Units.inchesToMeters(16), Units.inchesToMeters(45.5)},
        {207.99429321289062, 64.74365234375, Units.inchesToMeters(20), Units.inchesToMeters(45.5)},
        {216.6978759765625, 65.65827941894531, Units.inchesToMeters(24), Units.inchesToMeters(45.5)},
        {156.3396453857422, 57.33953094482422, Units.inchesToMeters(0), Units.inchesToMeters(49.5)},
        {166.42056274414062, 57.79549789428711, Units.inchesToMeters(4), Units.inchesToMeters(49.5)},
        {177.30941772460938, 58.38725280761719, Units.inchesToMeters(8), Units.inchesToMeters(49.5)},
        {186.7626953125, 59.280269622802734, Units.inchesToMeters(12), Units.inchesToMeters(49.5)},
        {
          196.08949279785156, 60.245914459228516, Units.inchesToMeters(16), Units.inchesToMeters(49.5)
        },
        {205.2306671142578, 61.16556930541992, Units.inchesToMeters(20), Units.inchesToMeters(49.5)},
        {213.36785888671875, 62.33674621582031, Units.inchesToMeters(24), Units.inchesToMeters(49.5)},
        {156.1303253173828, 54.62651443481445, Units.inchesToMeters(0), Units.inchesToMeters(53.5)},
        {165.8522491455078, 54.650146484375, Units.inchesToMeters(4), Units.inchesToMeters(53.5)},
        {175.65347290039062, 55.44529724121094, Units.inchesToMeters(8), Units.inchesToMeters(53.5)},
        {184.98977661132812, 56.34360885620117, Units.inchesToMeters(12), Units.inchesToMeters(53.5)},
        {193.63018798828125, 57.18233108520508, Units.inchesToMeters(16), Units.inchesToMeters(53.5)},
        {
          201.44790649414062, 58.028751373291016, Units.inchesToMeters(20), Units.inchesToMeters(53.5)
        },
        {209.90614318847656, 59.1672477722168, Units.inchesToMeters(24), Units.inchesToMeters(53.5)},
      };
    }
}