package org.firstinspires.ftc.teamcode.filters;

public abstract class Filter<I, O> {
    public abstract void update(I measurement);
    public abstract O value();
}
