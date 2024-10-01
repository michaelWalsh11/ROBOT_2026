package org.firstinspires.ftc.teamcode.AttemptAuto;

import javax.lang.model.type.ArrayType;
import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;

public class GraphReader
{
    private ArrayList<String> lines = new ArrayList<String>();
    private ArrayList<City> cities = new ArrayList<City>();

    public void readFile(String path)
    {
        try
        {
            File file = new File(path);
            Scanner scanner = new Scanner(file);
            while(scanner.hasNext())
            {
                String line = scanner.nextLine();
                lines.add(line);

            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
    public void createCityList()
    {
        for (String line : lines)
        {
            String lineData [] = line.split(", ");
            String name = lineData[0];
            double x = Double.valueOf(lineData[1]);
            double y = Double.valueOf(lineData[2]);
            City c = new City(name, x, y);
            cities.add(c);
        }
    }

    public void addNeighbors()
    {
        for (String line : lines)
        {
            String lineData [] = line.split(", ");
            City nodeCity = getCityFromName(lineData[0]);
            for (int x = 3; x <lineData.length; x+=2)
            {
                String name = lineData[x];
                double distance = Double.valueOf(lineData[x+1]);
                City c = getCityFromName(name);
                Neighbor n = new Neighbor(c, distance);
                nodeCity.addNeighbor(c, distance);
            }
        }
    }
    public City getCityFromName(String name)
    {
        for (City city : cities)
        {
            if(city.getName().equals(name))
            {
                return city;
            }
        }
        return null;
    }
    public ArrayList<City> getCities()
    {
        return cities;
    }
}
