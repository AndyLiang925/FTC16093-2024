package XCYOS;

import java.util.function.BooleanSupplier;

public class Wait4ConditionTask extends Task {
   BooleanSupplier condition;

   public Wait4ConditionTask(BooleanSupplier condition) {
      this.condition = condition;
   }

   @Override
   public void run() {
      if(condition.getAsBoolean()){
         status = Status.ENDED;
      }
   }
}
