package frc.robot.utils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ThreadPool {
  public static ExecutorService threadPool = Executors.newCachedThreadPool();

}
