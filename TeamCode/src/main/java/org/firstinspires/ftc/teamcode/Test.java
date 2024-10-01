package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AttemptAuto.City;
import org.firstinspires.ftc.teamcode.AttemptAuto.Path;
import org.firstinspires.ftc.teamcode.AttemptAuto.GraphReader;
import org.firstinspires.ftc.teamcode.AttemptAuto.Directions;
import org.firstinspires.ftc.teamcode.AttemptAuto.Neighbor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.List;

// **IMPORTANT** Units on map are 4 "units" = 1 inch **IMPORTANT** //
// **IMPORTANT** Units on map are 4 "units" = 1 inch **IMPORTANT** //
// **IMPORTANT** Units on map are 4 "units" = 1 inch **IMPORTANT** //
// **IMPORTANT** Units on map are 4 "units" = 1 inch **IMPORTANT** //

@Autonomous(name = "TestRouter", group = "Autonomous")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() {
        LinearOpModeBackgroundInfoCopy robot = new LinearOpModeBackgroundInfoCopy();
        HardwareMap HardwareMap;
        Pose2d pose = new Pose2d(0, 0, Math.toRadians(0));
        //MecanumDrive drive = new MecanumDrive(robot.hwMap, pose);
        //Guide API
        GraphReader reader = new GraphReader();

        reader.readFile("feildTest.txt");
        reader.createCityList();
        reader.addNeighbors();

        Path output = doStuffandDontDisplay("FiveFour", "TwoTwo", reader);
        Directions directions = new Directions(output, reader);

        waitForStart();

        //drive.actionBuilder(pose).lineToX(48);
        //testDrive(directions, robot);

        if (isStopRequested()) return;

    }

    //idk what the move commands are rn
    //this will work once I figure it out unless it tries to call multiple at once
    //then I will need to make a wait method to wait until the command is done being
    //completed and then have it find the next step although eventally I will need
    //a more efficent version this works for now to see if this actually works.
    public static void testDrive(Directions directions, LinearOpModeBackgroundInfoCopy robot) {
        for (String str: directions.getDirectionList())
        {
            double len = Double.parseDouble(str.substring(str.lastIndexOf(" "), str.length()));
            if (str.toLowerCase().contains("forward"))
            {
                robot.moveDistance(len, 0.5);
            }
            else if (str.toLowerCase().contains("Turn Left"))
            {
                robot.turnLeft(len, 0.5);
            }
            else if (str.toLowerCase().contains("Turn Right"))
            {
                robot.turnRight(len, 0.5);
            }
        }
    }

    public static Path doStuffandDontDisplay(String name1, String name2, GraphReader reader)
    {
        City startCity = reader.getCityFromName(name1);
        City endCity = reader.getCityFromName(name2);
        Path output = bfsTraversal(startCity, endCity, reader);
        //System.out.println(output);

        return output;
    }
//                                                                                                              penis in micheals butt
//     hehehehehehe
//    ive never been to africa ;)


    public static Path bfsTraversal(City startCity, City endCity, GraphReader map) {
        Path path = new Path();
        path.addCity(startCity);
        if (startCity.equals(endCity)) {
            return path;
        }

        ArrayList<Path> frontier = new ArrayList<Path>();
        frontier.add(path);
        ArrayList<City> explored = new ArrayList<City>();
        explored.add(startCity);

        while (!frontier.isEmpty()) {
            Path currentPath = frontier.remove(0);
            City lastCity = currentPath.getLastCity();
            //System.out.println(lastCity);
            explored.add(lastCity);
            ArrayList<City> neighbors = lastCity.getNeighbor();
            for (City neighbor : neighbors) {
                //System.out.println(neighbor);
                if (!explored.contains(neighbor) && !Path.frontierContainsCity(frontier, neighbor)) {
                    Path newPath = currentPath.getCopy();
                    newPath.addCity(neighbor);

                    if (neighbor.equals(endCity)) {
                        return newPath;
                    }

                    frontier.add(newPath);
                }
            }
        }

        return new Path();
    }
}
