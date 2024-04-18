package XCYOS;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class XCYOSCore {
    private static final ArrayList<Task> registeredTask = new ArrayList<>();
    private static OpMode opmode;
    private static List<LynxModule> allHubs;

    public static void addTask(Task task) {
        registeredTask.add(task);
    }

    public static void setUp(OpMode opmode) {
        XCYOSCore.opmode = opmode;
        LynxModuleUtil.ensureMinimumFirmwareVersion(opmode.hardwareMap);
        allHubs = opmode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
                        task.setUp();
                        task.setStatus(Task.Status.RUNNING);
                    case RUNNING:
                        task.run();
                }
            } else if (task.type == Task.Type.BASE) {
                task.run();
                task.setStatus(Task.Status.RUNNING);
            }
        }
        opmode.telemetry.update();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}
