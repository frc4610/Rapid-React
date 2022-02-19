package frc.robot.utils;

// Thread safe singleton

class Singleton {
  private static Singleton instance = null;
  public String info;

  private Singleton() {
    info = "Singleton";
  }

  public String getInfo() {
    return info;
  }

  public void setInfo(String info) {
    this.info = info;
  }

  public static Singleton get() {
    if (instance == null) {
      synchronized (Singleton.class) {
        if (instance == null) {
          instance = new Singleton();
        }
      }
    }
    return instance;
  }
}