package XCYOS;

public class TaskChainBuilder {
   Task base;
   Task head;
   Task branchingTask;

   public TaskChainBuilder add(Task task) {
      if (base == null) {
         base = task;
      } else {
         head.setNextTask(task);
         head = task;
      }
      return this;
   }

   public TaskChainBuilder addBranch(Task task) {
      if (branchingTask!=null){
         throw new IllegalArgumentException("create new branch when last branch not end");
      }
      branchingTask = head;
      head.setNextTask(task);
      head=task;
      return this;
   }

   public Task getBase() {
      return base;
   }

   public TaskChainBuilder end() {
      if (branchingTask != null) {
         head = branchingTask;
      }
      return this;
   }
}
