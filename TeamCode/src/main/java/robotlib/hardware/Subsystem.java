package robotlib.hardware;

public interface Subsystem {
    void start();
    void update();
    void stop();
    void updateMotorsAndServos();
}
