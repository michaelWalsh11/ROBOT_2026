package org.firstinspires.ftc.teamcode.AttemptAuto;

public class Neighbor
{
    private City city;
    private double distance;

    public Neighbor(City city, double distance)
    {
        this.city = city;
        this.distance = distance;
    }
    public City getCity()
    {
        return city;
    }
    public double getDistance()
    {
        return distance;
    }

}

