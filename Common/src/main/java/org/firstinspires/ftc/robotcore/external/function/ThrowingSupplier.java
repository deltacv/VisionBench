package org.firstinspires.ftc.robotcore.external.function;

public interface ThrowingSupplier<T, E extends Throwable> {
    T get() throws E;
}
