package org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism;

import java.util.List;
import org.sciborgs1155.lib.failure.HardwareFault;

public class EmptySubmech implements SubmechIO {

  public EmptySubmech() {}

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
