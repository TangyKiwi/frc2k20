package frc.robot.utilities;

import java.util.ArrayList;

public class RollingAverage 
{
    private int size;
    private double sum;
    private double [] arr;

    public RollingAverage(int _size)
    {
        size = _size;
        arr = new double[size];
        sum = 0;
    }

    public void add(double x)
    {
        sum -= arr[0];
        for(int i = 0; i < size - 1; i++)
        {
            arr[i] = arr[i + 1];
        }
        arr[size - 1] = x;
        sum += x;
    }

    public double getAverage()
    {
        return sum / size;
    }

    public void reset()
    {
        for(int i = 0; i < size; i++)
        {
           arr[i] = 0; 
        }
        sum = 0;
    }
}