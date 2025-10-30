using System;
using System.Collections.Generic;
using System.Text;

namespace _2D_Physics_Simulation
{
    public class ParticleCollisionEvent
    {
        public string particleID1;
        public string particleID2;
        public Vector positionOfCollision;
        public float timeOfCollision;

        public ParticleCollisionEvent(string particleID1, string particleID2, Vector position, float timeOfCollision)
        {
            this.particleID1 = particleID1;
            this.particleID2 = particleID2;
            this.positionOfCollision = position;
            this.timeOfCollision = timeOfCollision;
        }
    }
}
