package org.firstinspires.ftc.robotcore.external.function;

public interface InterruptableThrowingRunnable<E extends Throwable> {
    void run() throws E, InterruptedException;
}
