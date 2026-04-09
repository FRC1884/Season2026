package org.Griffins1884.frc2026.simulation.engine;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/** Deterministic broadphase and narrowphase collision detection. */
public final class CollisionSystem {
  private static final double EPSILON = 1e-6;

  public List<ContactState> detect(List<RigidBody> bodies, Set<Long> ignoredPairs) {
    List<BodyAabb> sorted = new ArrayList<>(bodies.size());
    for (RigidBody body : bodies) {
      sorted.add(new BodyAabb(body, computeAabb(body)));
    }
    sorted.sort(
        Comparator.comparingDouble((BodyAabb entry) -> entry.aabb.min().x())
            .thenComparingInt(entry -> entry.body.id()));

    List<ContactState> contacts = new ArrayList<>();
    for (int i = 0; i < sorted.size(); i++) {
      BodyAabb left = sorted.get(i);
      for (int j = i + 1; j < sorted.size(); j++) {
        BodyAabb right = sorted.get(j);
        if (right.aabb.min().x() > left.aabb.max().x()) {
          break;
        }
        if (!left.aabb.overlaps(right.aabb)) {
          continue;
        }
        if (ignoredPairs.contains(pairKey(left.body.id(), right.body.id()))
            || left.body.collisionGroup() == right.body.collisionGroup()) {
          continue;
        }
        if (!left.body.isDynamic() && !right.body.isDynamic()) {
          continue;
        }
        narrowphase(left.body, right.body).ifPresent(contacts::add);
      }
    }
    contacts.sort(
        Comparator.comparingInt(ContactState::bodyAId)
            .thenComparingInt(ContactState::bodyBId)
            .thenComparingDouble(contact -> contact.point().x())
            .thenComparingDouble(contact -> contact.point().y())
            .thenComparingDouble(contact -> contact.point().z()));
    return contacts;
  }

  private long pairKey(int a, int b) {
    int low = Math.min(a, b);
    int high = Math.max(a, b);
    return (((long) low) << 32) | (high & 0xffffffffL);
  }

  private java.util.Optional<ContactState> narrowphase(RigidBody bodyA, RigidBody bodyB) {
    CollisionShape shapeA = bodyA.shape();
    CollisionShape shapeB = bodyB.shape();
    return switch (shapeA.type()) {
      case SPHERE -> sphereAgainst(bodyA, bodyB, shapeA, shapeB);
      case BOX -> boxAgainst(bodyA, bodyB, shapeA, shapeB);
      case CYLINDER -> cylinderAgainst(bodyA, bodyB, shapeA, shapeB);
      case PLANE -> planeAgainst(bodyA, bodyB, shapeA, shapeB);
    };
  }

  private java.util.Optional<ContactState> sphereAgainst(
      RigidBody bodyA, RigidBody bodyB, CollisionShape shapeA, CollisionShape shapeB) {
    if (shapeB.type() == CollisionShape.ShapeType.PLANE) {
      return spherePlane(bodyA, (SphereShape) shapeA, bodyB, (PlaneShape) shapeB);
    }
    if (shapeB.type() == CollisionShape.ShapeType.SPHERE) {
      return sphereSphere(bodyA, (SphereShape) shapeA, bodyB, (SphereShape) shapeB);
    }
    if (shapeB.type() == CollisionShape.ShapeType.BOX) {
      return sphereBox(bodyA, (SphereShape) shapeA, bodyB, (BoxShape) shapeB);
    }
    return java.util.Optional.empty();
  }

  private java.util.Optional<ContactState> boxAgainst(
      RigidBody bodyA, RigidBody bodyB, CollisionShape shapeA, CollisionShape shapeB) {
    if (shapeB.type() == CollisionShape.ShapeType.PLANE) {
      return boxPlane(bodyA, (BoxShape) shapeA, bodyB, (PlaneShape) shapeB);
    }
    if (shapeB.type() == CollisionShape.ShapeType.BOX) {
      return boxBox(bodyA, (BoxShape) shapeA, bodyB, (BoxShape) shapeB);
    }
    if (shapeB.type() == CollisionShape.ShapeType.SPHERE) {
      return sphereBox(bodyB, (SphereShape) shapeB, bodyA, (BoxShape) shapeA);
    }
    return java.util.Optional.empty();
  }

  private java.util.Optional<ContactState> cylinderAgainst(
      RigidBody bodyA, RigidBody bodyB, CollisionShape shapeA, CollisionShape shapeB) {
    if (shapeB.type() == CollisionShape.ShapeType.PLANE) {
      return cylinderPlane(bodyA, (CylinderShape) shapeA, bodyB, (PlaneShape) shapeB);
    }
    return java.util.Optional.empty();
  }

  private java.util.Optional<ContactState> planeAgainst(
      RigidBody bodyA, RigidBody bodyB, CollisionShape shapeA, CollisionShape shapeB) {
    return switch (shapeB.type()) {
      case SPHERE -> spherePlane(bodyB, (SphereShape) shapeB, bodyA, (PlaneShape) shapeA);
      case BOX -> boxPlane(bodyB, (BoxShape) shapeB, bodyA, (PlaneShape) shapeA);
      case CYLINDER -> cylinderPlane(bodyB, (CylinderShape) shapeB, bodyA, (PlaneShape) shapeA);
      case PLANE -> java.util.Optional.empty();
    };
  }

  private java.util.Optional<ContactState> sphereSphere(
      RigidBody bodyA, SphereShape sphereA, RigidBody bodyB, SphereShape sphereB) {
    Vec3 delta = bodyB.position().subtract(bodyA.position());
    double distance = delta.norm();
    double radii = sphereA.radius() + sphereB.radius();
    if (distance >= radii) {
      return java.util.Optional.empty();
    }
    Vec3 normalFromAToB = distance > EPSILON ? delta.divide(distance) : Vec3.UNIT_X;
    Vec3 point =
        bodyA.position().add(normalFromAToB.scale(sphereA.radius() - (0.5 * (radii - distance))));
    return java.util.Optional.of(
        orderedContact(bodyA, bodyB, point, normalFromAToB, radii - distance));
  }

  private java.util.Optional<ContactState> spherePlane(
      RigidBody sphereBody, SphereShape sphere, RigidBody planeBody, PlaneShape plane) {
    Vec3 planeNormal = planeWorldNormal(planeBody, plane);
    Vec3 planePoint = planeWorldPoint(planeBody, plane);
    double signedDistance = sphereBody.position().subtract(planePoint).dot(planeNormal);
    double penetration = sphere.radius() - signedDistance;
    if (penetration <= 0.0) {
      return java.util.Optional.empty();
    }
    Vec3 point =
        sphereBody.position().subtract(planeNormal.scale(signedDistance - (0.5 * penetration)));
    return java.util.Optional.of(
        orderedContact(sphereBody, planeBody, point, planeNormal.negate(), penetration));
  }

  private java.util.Optional<ContactState> sphereBox(
      RigidBody sphereBody, SphereShape sphere, RigidBody boxBody, BoxShape box) {
    Vec3 localCenter = boxBody.localSpaceVector(sphereBody.position().subtract(boxBody.position()));
    Vec3 closestLocal =
        new Vec3(
            clamp(localCenter.x(), -box.halfX(), box.halfX()),
            clamp(localCenter.y(), -box.halfY(), box.halfY()),
            clamp(localCenter.z(), -box.halfZ(), box.halfZ()));
    Vec3 closestWorld = boxBody.worldSpacePoint(closestLocal);
    Vec3 delta = sphereBody.position().subtract(closestWorld);
    double distance = delta.norm();
    if (distance >= sphere.radius()) {
      return java.util.Optional.empty();
    }
    Vec3 normalFromSphereToBox =
        distance > EPSILON
            ? delta.divide(distance).negate()
            : boxBody.worldSpaceVector(Vec3.UNIT_X);
    return java.util.Optional.of(
        orderedContact(
            sphereBody, boxBody, closestWorld, normalFromSphereToBox, sphere.radius() - distance));
  }

  private java.util.Optional<ContactState> boxPlane(
      RigidBody boxBody, BoxShape box, RigidBody planeBody, PlaneShape plane) {
    Vec3 normal = planeWorldNormal(planeBody, plane);
    Vec3 support = boxSupportPoint(boxBody, box, normal.negate());
    double distance = support.subtract(planeWorldPoint(planeBody, plane)).dot(normal);
    if (distance >= 0.0) {
      return java.util.Optional.empty();
    }
    return java.util.Optional.of(
        orderedContact(boxBody, planeBody, support, normal.negate(), -distance));
  }

  private java.util.Optional<ContactState> cylinderPlane(
      RigidBody cylinderBody, CylinderShape cylinder, RigidBody planeBody, PlaneShape plane) {
    Vec3 normal = planeWorldNormal(planeBody, plane);
    Vec3 axis = cylinderBody.worldSpaceVector(Vec3.UNIT_Y).normalize();
    double axisDot = Math.abs(normal.dot(axis));
    double radialTerm = cylinder.radius() * Math.sqrt(Math.max(0.0, 1.0 - (axisDot * axisDot)));
    double extent = (cylinder.halfHeight() * axisDot) + radialTerm;
    double signedDistance =
        cylinderBody.position().subtract(planeWorldPoint(planeBody, plane)).dot(normal);
    double penetration = extent - signedDistance;
    if (penetration <= 0.0) {
      return java.util.Optional.empty();
    }
    Vec3 point =
        cylinderBody.position().subtract(normal.scale(signedDistance - (0.5 * penetration)));
    return java.util.Optional.of(
        orderedContact(cylinderBody, planeBody, point, normal.negate(), penetration));
  }

  private java.util.Optional<ContactState> boxBox(
      RigidBody bodyA, BoxShape boxA, RigidBody bodyB, BoxShape boxB) {
    Vec3[] axesA = basis(bodyA.orientation());
    Vec3[] axesB = basis(bodyB.orientation());
    double[][] rotation = new double[3][3];
    double[][] absRotation = new double[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        rotation[i][j] = axesA[i].dot(axesB[j]);
        absRotation[i][j] = Math.abs(rotation[i][j]) + EPSILON;
      }
    }
    Vec3 translationWorld = bodyB.position().subtract(bodyA.position());
    Vec3 translation =
        new Vec3(
            translationWorld.dot(axesA[0]),
            translationWorld.dot(axesA[1]),
            translationWorld.dot(axesA[2]));
    double[] extentsA = {boxA.halfX(), boxA.halfY(), boxA.halfZ()};
    double[] extentsB = {boxB.halfX(), boxB.halfY(), boxB.halfZ()};

    double minOverlap = Double.POSITIVE_INFINITY;
    Vec3 minAxis = Vec3.UNIT_X;

    for (int i = 0; i < 3; i++) {
      double ra = extentsA[i];
      double rb =
          (extentsB[0] * absRotation[i][0])
              + (extentsB[1] * absRotation[i][1])
              + (extentsB[2] * absRotation[i][2]);
      double overlap = ra + rb - Math.abs(translation.get(i));
      if (overlap <= 0.0) {
        return java.util.Optional.empty();
      }
      if (overlap < minOverlap) {
        minOverlap = overlap;
        minAxis = translation.get(i) < 0.0 ? axesA[i].negate() : axesA[i];
      }
    }

    for (int i = 0; i < 3; i++) {
      double ra =
          (extentsA[0] * absRotation[0][i])
              + (extentsA[1] * absRotation[1][i])
              + (extentsA[2] * absRotation[2][i]);
      double rb = extentsB[i];
      double projection =
          Math.abs(
              (translation.get(0) * rotation[0][i])
                  + (translation.get(1) * rotation[1][i])
                  + (translation.get(2) * rotation[2][i]));
      double overlap = ra + rb - projection;
      if (overlap <= 0.0) {
        return java.util.Optional.empty();
      }
      if (overlap < minOverlap) {
        minOverlap = overlap;
        minAxis =
            ((translation.get(0) * rotation[0][i])
                        + (translation.get(1) * rotation[1][i])
                        + (translation.get(2) * rotation[2][i]))
                    < 0.0
                ? axesB[i].negate()
                : axesB[i];
      }
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Vec3 axis = axesA[i].cross(axesB[j]);
        double axisNorm = axis.norm();
        if (axisNorm < EPSILON) {
          continue;
        }
        Vec3 normalizedAxis = axis.divide(axisNorm);
        double ra =
            extentsA[(i + 1) % 3] * absRotation[(i + 2) % 3][j]
                + extentsA[(i + 2) % 3] * absRotation[(i + 1) % 3][j];
        double rb =
            extentsB[(j + 1) % 3] * absRotation[i][(j + 2) % 3]
                + extentsB[(j + 2) % 3] * absRotation[i][(j + 1) % 3];
        double projection = Math.abs(translationWorld.dot(normalizedAxis));
        double overlap = ra + rb - projection;
        if (overlap <= 0.0) {
          return java.util.Optional.empty();
        }
        if (overlap < minOverlap) {
          minOverlap = overlap;
          minAxis =
              translationWorld.dot(normalizedAxis) < 0.0 ? normalizedAxis.negate() : normalizedAxis;
        }
      }
    }

    Vec3 supportA = boxSupportPoint(bodyA, boxA, minAxis.negate());
    Vec3 supportB = boxSupportPoint(bodyB, boxB, minAxis);
    Vec3 point = supportA.add(supportB).scale(0.5);
    return java.util.Optional.of(orderedContact(bodyA, bodyB, point, minAxis, minOverlap));
  }

  private Vec3 boxSupportPoint(RigidBody body, BoxShape box, Vec3 direction) {
    Vec3 localDirection = body.localSpaceVector(direction);
    Vec3 localSupport =
        new Vec3(
            localDirection.x() >= 0.0 ? box.halfX() : -box.halfX(),
            localDirection.y() >= 0.0 ? box.halfY() : -box.halfY(),
            localDirection.z() >= 0.0 ? box.halfZ() : -box.halfZ());
    return body.worldSpacePoint(localSupport);
  }

  private ContactState orderedContact(
      RigidBody bodyA, RigidBody bodyB, Vec3 point, Vec3 normalFromAToB, double penetration) {
    RigidBody first = bodyA;
    RigidBody second = bodyB;
    Vec3 contactNormal = normalFromAToB;
    if (bodyA.id() > bodyB.id()) {
      first = bodyB;
      second = bodyA;
      contactNormal = normalFromAToB.negate();
    }
    double friction =
        Math.sqrt(first.material().frictionCoefficient() * second.material().frictionCoefficient());
    double restitution =
        Math.max(
            first.material().restitutionCoefficient(), second.material().restitutionCoefficient());
    return new ContactState(
        first.id(), second.id(), point, contactNormal, penetration, friction, restitution);
  }

  public Aabb computeAabb(RigidBody body) {
    CollisionShape shape = body.shape();
    return switch (shape.type()) {
      case SPHERE -> {
        SphereShape sphere = (SphereShape) shape;
        Vec3 radius = new Vec3(sphere.radius(), sphere.radius(), sphere.radius());
        yield new Aabb(body.position().subtract(radius), body.position().add(radius));
      }
      case BOX -> {
        BoxShape box = (BoxShape) shape;
        Vec3 extent = orientedBoxAabbExtent(body.orientation(), box);
        yield new Aabb(body.position().subtract(extent), body.position().add(extent));
      }
      case CYLINDER -> {
        CylinderShape cylinder = (CylinderShape) shape;
        Vec3 extent = orientedCylinderAabbExtent(body.orientation(), cylinder);
        yield new Aabb(body.position().subtract(extent), body.position().add(extent));
      }
      case PLANE -> new Aabb(new Vec3(-1e6, -1e6, -1e6), new Vec3(1e6, 1e6, 1e6));
    };
  }

  private Vec3 orientedBoxAabbExtent(Quat orientation, BoxShape box) {
    Vec3[] basis = basis(orientation);
    return new Vec3(
        Math.abs(basis[0].x()) * box.halfX()
            + Math.abs(basis[1].x()) * box.halfY()
            + Math.abs(basis[2].x()) * box.halfZ(),
        Math.abs(basis[0].y()) * box.halfX()
            + Math.abs(basis[1].y()) * box.halfY()
            + Math.abs(basis[2].y()) * box.halfZ(),
        Math.abs(basis[0].z()) * box.halfX()
            + Math.abs(basis[1].z()) * box.halfY()
            + Math.abs(basis[2].z()) * box.halfZ());
  }

  private Vec3 orientedCylinderAabbExtent(Quat orientation, CylinderShape cylinder) {
    Vec3 axis = orientation.axisY().normalize();
    double axisX = Math.abs(axis.x());
    double axisY = Math.abs(axis.y());
    double axisZ = Math.abs(axis.z());
    return new Vec3(
        cylinder.halfHeight() * axisX
            + cylinder.radius() * Math.sqrt(Math.max(0.0, 1.0 - (axisX * axisX))),
        cylinder.halfHeight() * axisY
            + cylinder.radius() * Math.sqrt(Math.max(0.0, 1.0 - (axisY * axisY))),
        cylinder.halfHeight() * axisZ
            + cylinder.radius() * Math.sqrt(Math.max(0.0, 1.0 - (axisZ * axisZ))));
  }

  private Vec3 planeWorldNormal(RigidBody planeBody, PlaneShape plane) {
    return planeBody.worldSpaceVector(plane.localNormal()).normalize();
  }

  private Vec3 planeWorldPoint(RigidBody planeBody, PlaneShape plane) {
    return planeBody.position().add(planeWorldNormal(planeBody, plane).scale(plane.offset()));
  }

  private Vec3[] basis(Quat orientation) {
    return new Vec3[] {orientation.axisX(), orientation.axisY(), orientation.axisZ()};
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private record BodyAabb(RigidBody body, Aabb aabb) {}
}
