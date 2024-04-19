package XCYOS;

public class TaskChainBuilder {
   Task base;
   Task head;
   Task branchingTask;

   public TaskChainBuilder start(){
      base=null;
      head=null;
      branchingTask=null;
      return this;
   }

   public TaskChainBuilder add(Task task) {
      if (base == null) {
         base = task;
      } else {
         head.setNextTask(task);
      }
      task.setStatus(Task.Status.PENDING);
      head = task;
      return this;
   }

   public TaskChainBuilder add(Runnable runnable){
      add(new Task() {
         @Override
         public void run() {
            runnable.run();
            status=Status.ENDED;
         }
      });
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
