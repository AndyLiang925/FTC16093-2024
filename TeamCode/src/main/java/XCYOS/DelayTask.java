package XCYOS;

public class DelayTask extends Task {
   private final long timeEnd;

   public DelayTask(int time_ms) {
      timeEnd = System.currentTimeMillis() + time_ms;
   }

   @Override
   public void run() {
      if (timeEnd > System.currentTimeMillis()) {
         status = Status.ENDED;
      }
   }
}
