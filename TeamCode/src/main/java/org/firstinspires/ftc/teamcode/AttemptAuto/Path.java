package org.firstinspires.ftc.teamcode.AttemptAuto;

import java.util.ArrayList;

public class Path
{
    private ArrayList<City> route = new ArrayList<City>();
    private double length = 0;

    public Path()
    {

    }
    public static boolean frontierContainsCity(ArrayList<Path> frontier, City city)
    {
        for (Path path : frontier)
        {
            if (path.getLastCity().equals(city))
            {
                return true;
            }
        }
        return false;
    }

    public Path getCopy()
    {
        Path p = new Path();
        for (City c : route)
        {
            p.addCity(c);
        }
        return p;
    }

    public Path(Path p, City c)
    {
        for (City city : p.getRoute())
        {
            addCity(city);
        }
        addCity(c);
    }

    public void addCity(City c)
    {
        if (route.isEmpty())
        {
            route.add(c);
        }
        else
        {
            City last = getLastCity();
            for(Neighbor n : last.getNeighbors())
            {
                if (n.getCity().equals(c))
                {
                    length += n.getDistance();
                    route.add(c);
                }
            }
        }
    }
    public City getLastCity()
    {
        return route.get(route.size()-1);
    }
    public double getLength()
    {
        return length;
    }
    public ArrayList<City> getRoute()
    {
        return route;
    }
    public String toString()
    {
        String out = String.valueOf(length) + ", ";
        for (City c : route)
        {
            out += c.getName() + ", ";
        }
        return out.substring(0, out.length()-2);
    }
    public ArrayList<City> pathToCity(Path path, GraphReader reader)
    {
        ArrayList<City> city = new ArrayList<City>();

        String p = path.toString();
        p = p.replace(",", "");
        String [] arrX = p.split(" ");

        for (int x = 1; x < arrX.length; x++)
        {
            city.add(reader.getCityFromName(arrX[x]));
        }


        return city;
    }
}

