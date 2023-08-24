package org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism;

import java.util.List;
import org.sciborgs1155.lib.failure.HardwareFault;

public class SimSubmech implements SubmechIO {

  public SimSubmech() {}

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
  }

  @Override
  public List<HardwareFault> getFaults() {
    return List.of();
  }
}
