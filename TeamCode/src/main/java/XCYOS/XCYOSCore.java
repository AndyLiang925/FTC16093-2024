package XCYOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Iterator;

public class XCYOSCore {
   private static final ArrayList<Task> registeredTask = new ArrayList<>();
   private static Telemetry telemetry;

   public static void addTask(Task task) {
      registeredTask.add(task);
   }

   public static void setTelemetry(Telemetry telemetry) {
      XCYOSCore.telemetry = telemetry;
   }

   public static boolean checkComponents(Component[] componentUsed) {
      for (Component c : componentUsed) {
         if (c.isOccupied()) return false;
      }
      return true;
   }

   public static void update() {
      Iterator<Task> iter = registeredTask.iterator();
      while (iter.hasNext()) {
         Task task = iter.next();
         if (task.type == Task.Type.INSTANT) {
            switch (task.status) {
               case ENDED:
                  task.end();
                  iter.remove();
                  registeredTask.addAll(task.getNextTasks());
               case PENDING:
                  if (checkComponents(task.componentsUsed)) {
                     task.setUp();
                     task.setStatus(Task.Status.RUNNING);
                  } else {
                     telemetry.addLine("ERROR: component");
                  }
               case RUNNING:
                  task.run();
            }
         } else if (task.type == Task.Type.BASE) {
            if (checkComponents(task.componentsUsed)) {
               task.run();
               task.setStatus(Task.Status.RUNNING);
            } else {
               task.setStatus(Task.Status.PENDING);
            }
         }
      }
      telemetry.update();
   }
}
