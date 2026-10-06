package org.firstinspires.ftc.robotcore.external.function;

public interface ThrowingRunnable<E extends Throwable> {
    void run() throws E;
}
