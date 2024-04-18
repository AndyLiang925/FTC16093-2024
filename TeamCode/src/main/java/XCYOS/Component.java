package XCYOS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public abstract class Component {
   private boolean isOccupied =false;
   abstract void init(HardwareMap hardwareMap);

   public void setOccupied(boolean occupied) {
      isOccupied = occupied;
   }

   boolean isOccupied(){
      return isOccupied;
   }
}
