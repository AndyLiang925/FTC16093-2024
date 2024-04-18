package XCYOS;

import java.util.ArrayList;

public abstract class Task implements Runnable {
   protected Type type = Type.INSTANT;
   protected Status status = Status.PENDING;
   private final ArrayList<Task> nextTask=new ArrayList<>(1);

   public void setUp() {

   }

   public void setNextTask(Task task){
      nextTask.add(task);
   }

   public ArrayList<Task> getNextTasks(){
      return nextTask;
   }

   public void end() {

   }

   public void setStatus(Status status) {
      this.status = status;
   }

   public void setType(Type type) {
      this.type = type;
   }

   public enum Type {
      BASE, INSTANT
   }

   public enum Status {
      PENDING, RUNNING, ENDED
   }
}
