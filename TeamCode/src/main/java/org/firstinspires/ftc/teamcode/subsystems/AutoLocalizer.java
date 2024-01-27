package org.firstinspires.ftc.teamcode.subsystems;

public class AutoLocalizer {

    private final Odometry odometry;
    private final IMU_Controller imu;

    private final double ticksPerInchPar;  // TODO: Tune for parallel deadwheels
    private final double ticksPerInchPerp; // TODO: Tune for lateral deadwheel

    private final Position startingPosition;

    private Position currentPosition;

    private double lastTicks = 0;
    private double lastHeading = 0;

    public AutoLocalizer(Odometry odometry, IMU_Controller imu, double ticksPerInchPar, double ticksPerInchPerp, Position startingPosition) {
        this.odometry = odometry;
        this.imu = imu;
        this.ticksPerInchPar = ticksPerInchPar;
        this.ticksPerInchPerp = ticksPerInchPerp;
        this.startingPosition = startingPosition;
        lastHeading = imu.getHeading();
    }

    private double ticksToInchesPar(double ticks) {
        return ticks * ticksPerInchPar;
    }

    private double ticksToInchesPerp(double ticks) {
        return ticks / ticksPerInchPerp;
    }

    private double calculateChangeInX(double deltaTicks) {
        // Calculate the change in x (0 Degrees direction is considered positive)
        // Coordinate system is viewed from right side of the truss with pixel board to right and plane zone to left
        return deltaTicks * Math.cos(Math.toRadians(imu.getHeading()));
    }

    private double calculateChangeInY(double deltaTicks) {
        // Calculate the change in y (90 Degrees direction is considered positive)
        // Coordinate system is viewed from right side of the truss with pixel board to right and plane zone to left
        return deltaTicks * Math.sin(Math.toRadians(imu.getHeading()));
    }

    private double calculateChangeInHeading() {
        // Implement based on your robot's kinematics
        return imu.getHeading() - lastHeading;  // Placeholder, you may need to adjust this
    }

    public void calculateCurrentPose() {

        double changeInX = calculateChangeInX(odometry.getParTicks() - lastTicks);
        double changeInY = calculateChangeInY(odometry.getParTicks() - lastTicks);
        double changeInHeading = calculateChangeInHeading();  // Implement this method


        double x = startingPosition.getX() + changeInX;
        double y = startingPosition.getY() + changeInY;
        double heading = startingPosition.getHeading() + changeInHeading;

        lastTicks = odometry.getParTicks();

        currentPosition = new Position(x, y, heading);
    }

    public Position getCurrentPosition() {
        return currentPosition;
    }
}
