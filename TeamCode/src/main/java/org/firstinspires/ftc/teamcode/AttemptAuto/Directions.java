package org.firstinspires.ftc.teamcode.AttemptAuto;

import java.util.ArrayList;

/**
 * A lot of the math in this class is AI generated due to the lack of knowledge and
 * in the interest of saving time, so I can hopfully figure out how to implement this into
 * a robot before the season starts. Sorry I'll look back at this later and try to understand it
 * better. - Michael Walsh
 */

public class Directions {
    private ArrayList<String> directionList;

    public Directions(Path path, GraphReader reader) {
        this.directionList = new ArrayList<>();
        generateDirections(path, reader);
    }



    private void generateDirections(Path path, GraphReader reader) {
        ArrayList<City> cities = path.pathToCity(path, reader);

        double accumulatedDistance = 0;

        for (int i = 0; i < cities.size() - 1; i++) {
            City currentCity = cities.get(i);
            City nextCity = cities.get(i + 1);
            City previousCity = (i > 0) ? cities.get(i - 1) : null;

            double distance = calculateDistance(currentCity, nextCity);

            // If there's a previous city, calculate the turn angle
            if (previousCity != null) {
                double angle = calculateTurnAngle(previousCity, currentCity, nextCity);

                if (Math.abs(angle) > 1e-5) {
                    if (accumulatedDistance > 0) {
                        directionList.add("Forward " + Math.round(accumulatedDistance) + " units");
                        accumulatedDistance = 0;
                    }
                    String turnDirection = (angle > 0) ? "Right" : "Left";
                    directionList.add("Turn " + turnDirection + " " + Math.abs(angle) + " degrees");
                }
            }

            accumulatedDistance += distance;
        }

        if (accumulatedDistance > 0) {
            directionList.add("Forward " + Math.round(accumulatedDistance) + " units");
        }
    }

    private double calculateDistance(City from, City to) {
        double dx = to.getxPos() - from.getxPos();
        double dy = to.getyPos() - from.getyPos();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double calculateTurnAngle(City previous, City current, City next) {

        double dx1 = current.getxPos() - previous.getxPos();
        double dy1 = current.getyPos() - previous.getyPos();

        double dx2 = next.getxPos() - current.getxPos();
        double dy2 = next.getyPos() - current.getyPos();

        double angle1 = Math.toDegrees(Math.atan2(dy1, dx1));
        double angle2 = Math.toDegrees(Math.atan2(dy2, dx2));

        double angle = angle2 - angle1;

        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    public ArrayList<String> getDirectionList() {
        return directionList;
    }

    public void printDirections() {
        for (String direction : directionList) {
            System.out.println(direction);
        }
    }

}

