package org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism;

import java.util.List;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;

public class RealSubmech implements SubmechIO {

  public RealSubmech() {}

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub

  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create().register("label", false).build();
  }
}
