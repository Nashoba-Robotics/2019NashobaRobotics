package edu.nr.robotics.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChoosers {

    public static SendableChooser<SandstormType> sandstormTypeChooser = new SendableChooser<>();
    public static SendableChooser<StartPos> autoStartPosChooser = new SendableChooser<>();
    public static SendableChooser<GamePiece> autoGamePiece1Chooser = new SendableChooser<>();
    public static SendableChooser<GamePiece> autoGamePiece2Chooser = new SendableChooser<>();
    public static SendableChooser<Platform> autoPlatformChooser = new SendableChooser<>();
    public static SendableChooser<Destination> autoDestination1Chooser = new SendableChooser<>();
    public static SendableChooser<Destination2> autoDestination2Chooser = new SendableChooser<>();


    public enum SandstormType {
        auto, driver;
    }

    public enum StartPos {
        left, middle, right;
    }

    public enum GamePiece {
        hatch, cargo;
    }

    public enum Platform {
        yes, no;
    }

    public enum Destination {
        rocketFront, rocketBack, cargoShipSide, cargoShipFrontLeft, cargoShipFrontRight;
    }

    public enum Destination2 {
        rocket, cargoShip;
    }

}