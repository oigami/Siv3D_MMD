#include "../include/BulletPhysics.h"

namespace s3d_bullet {
  BulletPhysics getBulletPhysics() {
    static BulletPhysics bulletPhysics(Float3(0.f, -9.8f, 0.f));
    return bulletPhysics;
  }
}
