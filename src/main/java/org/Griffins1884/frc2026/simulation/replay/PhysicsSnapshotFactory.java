package org.Griffins1884.frc2026.simulation.replay;

import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.ContactPointState;
import org.Griffins1884.frc2026.simulation.contracts.RigidBodyState;
import org.Griffins1884.frc2026.simulation.contracts.WheelContactState;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.engine.ContactState;
import org.Griffins1884.frc2026.simulation.engine.RigidBody;
import org.Griffins1884.frc2026.simulation.engine.WheelContactTelemetry;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;

/** Converts local physics simulation state into replayable world snapshots. */
public final class PhysicsSnapshotFactory {
  private PhysicsSnapshotFactory() {}

  public static WorldSnapshot fromSimulation(
      LocalSwervePhysicsSimulation simulation, long simTimeNanos, int stepId) {
    List<RigidBodyState> rigidBodyStates =
        simulation.getBodies().stream().map(PhysicsSnapshotFactory::bodyState).toList();
    List<ContactPointState> contactStates =
        simulation.getLastContacts().stream().map(PhysicsSnapshotFactory::contactState).toList();
    List<WheelContactState> wheelContactStates =
        simulation.getWheelContactTelemetry().stream()
            .map(PhysicsSnapshotFactory::wheelContactState)
            .toList();
    return new WorldSnapshot(
        simTimeNanos,
        stepId,
        false,
        false,
        null,
        null,
        simulation.getGamePiecePoses(),
        rigidBodyStates,
        contactStates,
        wheelContactStates);
  }

  private static RigidBodyState bodyState(RigidBody body) {
    return new RigidBodyState(
        body.id(),
        body.name(),
        body.shape().type().name(),
        body.isDynamic(),
        body.position().x(),
        body.position().y(),
        body.position().z(),
        body.orientation().w(),
        body.orientation().x(),
        body.orientation().y(),
        body.orientation().z(),
        body.linearVelocity().x(),
        body.linearVelocity().y(),
        body.linearVelocity().z(),
        body.angularVelocity().x(),
        body.angularVelocity().y(),
        body.angularVelocity().z());
  }

  private static ContactPointState contactState(ContactState contact) {
    return new ContactPointState(
        contact.bodyAId(),
        contact.bodyBId(),
        contact.point().x(),
        contact.point().y(),
        contact.point().z(),
        contact.normal().x(),
        contact.normal().y(),
        contact.normal().z(),
        contact.penetrationDepth(),
        contact.accumulatedNormalImpulse(),
        contact.accumulatedTangentImpulse1(),
        contact.accumulatedTangentImpulse2());
  }

  private static WheelContactState wheelContactState(WheelContactTelemetry telemetry) {
    return new WheelContactState(
        telemetry.wheelIndex(),
        telemetry.contacting(),
        telemetry.point().x(),
        telemetry.point().y(),
        telemetry.point().z(),
        telemetry.normal().x(),
        telemetry.normal().y(),
        telemetry.normal().z(),
        telemetry.normalForceNewtons(),
        telemetry.slipRatio(),
        telemetry.slipAngleRadians(),
        telemetry.longitudinalForce().x(),
        telemetry.longitudinalForce().y(),
        telemetry.longitudinalForce().z(),
        telemetry.lateralForce().x(),
        telemetry.lateralForce().y(),
        telemetry.lateralForce().z());
  }
}
