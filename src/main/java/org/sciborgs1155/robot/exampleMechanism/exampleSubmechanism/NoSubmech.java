package org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism;

import java.util.List;
import org.sciborgs1155.lib.failure.HardwareFault;

public class NoSubmech implements SubmechIO {

  public NoSubmech() {}

  @Override
  public boolean condition() {
    return false;
  }

  @Override
  public List<HardwareFault> getFaults() {
    // TODO Auto-generated method stub
    return List.of();
  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
  }
}
