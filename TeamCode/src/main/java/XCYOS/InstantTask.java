package XCYOS;

public class InstantTask extends Task{
    Runnable thing;
    public InstantTask(Runnable runnable){
        super();
        thing=runnable;
    }
    @Override
    public void run() {
        thing.run();
        status=Status.ENDED;
    }
}
