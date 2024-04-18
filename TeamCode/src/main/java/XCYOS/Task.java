package XCYOS;

import java.util.ArrayList;

public abstract class Task implements Runnable {
   protected Type type = Type.INSTANT;
   protected Status status = Status.PENDING;
   protected Component[] componentsUsed = new Component[0];
   private final ArrayList<Task> nextTask=new ArrayList<>(1);

   public void setUp() {
      for (Component c : componentsUsed) {
         c.setOccupied(true);
      }
   }

   public void setNextTask(Task task){
      nextTask.add(task);
   }

   public ArrayList<Task> getNextTasks(){
      return nextTask;
   }

   public void end() {
      for (Component c : componentsUsed) {
         c.setOccupied(false);
      }
   }

   public void setComponentsUsed(Component... componentsUsed) {
      this.componentsUsed = componentsUsed;
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
