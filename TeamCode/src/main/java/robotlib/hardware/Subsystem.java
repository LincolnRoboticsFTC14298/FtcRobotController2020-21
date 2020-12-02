package robotlib.hardware;

public abstract class Subsystem {
    public void init_loop() {

    }

    public abstract void start();
    public abstract void update();
    public abstract void stop();
    public abstract void updateMotorsAndServos();
}
