package org.firstinspires.ftc.griffins;

import java.util.LinkedList;
import java.util.Queue;

/**
 * Created by David on 10/27/2016.
 */
public class LowPassFilter {

    private int capacity;
    private Queue<Double> values;
    private double processedValue;

    public LowPassFilter(int capacity) {
        this.capacity = capacity;
        values = new LinkedList<>();
        processedValue = 0;
    }

    public double addValue(double newValue) {
        values.add(newValue);

        if (values.size() > capacity) {
            double removed = values.remove();
            processedValue -= removed / capacity;
            processedValue += newValue / capacity;
        } else {
            double sum = 0;
            for (double value : values) {
                sum += value;
            }

            processedValue = sum / values.size();
        }

        return processedValue;
    }

    public boolean atCapacity() {
        return values.size() == capacity;
    }

    public double getProcessedValue() {
        return processedValue;
    }
}
