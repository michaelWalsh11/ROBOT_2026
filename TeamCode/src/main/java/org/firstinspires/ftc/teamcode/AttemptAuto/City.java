package org.firstinspires.ftc.teamcode.AttemptAuto;

import java.util.ArrayList;

public class City
{
    private String name;
    private double xPos;
    private double yPos;
    private ArrayList<City> neighbors = new ArrayList<City>();

    private ArrayList<Neighbor> neighbor = new ArrayList<Neighbor>();
    private boolean visited;

    public City(String name, double xPos, double yPos)
    {
        this.name = name;
        this.xPos = xPos;
        this.yPos = yPos;
    }
    public String getName()
    {
        return name;
    }
    public double getxPos()
    {
        return xPos;
    }
    public double getyPos()
    {
        return yPos;
    }

    public ArrayList<Neighbor> getNeighbors() {
        return neighbor;
    }

    public ArrayList<City> getNeighbor()
    {
        return neighbors;
    }

    public boolean isVisited() {
        return visited;
    }

    public void setVisited(boolean visited) {
        this.visited = visited;
    }

    public boolean equals(City c)
    {
        return name.equals(c.getName());
    }

    public void addNeighbor(City c, double distance) {
        Neighbor neight = new Neighbor(c, distance);
        neighbor.add(neight);
        neighbors.add(c);
    }

    public String toString() {
        return String.format(name);
    }

}
