using System;
using System.Collections.Generic;
using System.Drawing;

namespace _2D_Physics_Simulation
{
    public class Simulation
    {
        private Particle[] particles;
        private int numberOfParticles;
        public float collisionCalcFreq = 10;
        private float collisionCheckTime;
        private List<ParticleCollisionEvent> particleCollisionEventsToCome;

        public int GetNumberOfParticles() { return numberOfParticles; }

        private float time;

        public float GetTime() { return time; }

        public Simulation(int maxParticles)
        {
            particles = new Particle[maxParticles];
            numberOfParticles = 0;

            time = 0;
            particleCollisionEventsToCome = new List<ParticleCollisionEvent>();
            collisionCheckTime = 1f/collisionCalcFreq;
        }

        public void AddParticle(Particle newParticle)
        {
            particles[numberOfParticles] = newParticle;
            numberOfParticles++;
        }

        public Vector GetParticlePosition(int particleIndex)
        {
            if(particles[particleIndex] == null)
            {
                throw (new Exception("Particle at index: " + particleIndex + " does not exist."));
            }

            return particles[particleIndex].GetPosition();
        }

        public Color GetParticleColor(int particleIndex)
        {
            if (particles[particleIndex] == null)
            {
                throw (new Exception("Particle at index: " + particleIndex + " does not exist."));
            }

            return particles[particleIndex].GetColor();
        }

        public void MoveTime(float moveTime)
        {
            Console.WriteLine("Current time: " + time);
            for(int i = 0; i < numberOfParticles; i++)
            {
                particles[i].ApplyResultantForce(moveTime);
                particles[i].UpdatePosition(moveTime);
                Console.WriteLine("Particle " + particles[i].GetID() + " position: " + particles[i].GetPosition().GetMagnitude());
            }
            time += moveTime;

            //Check if any collisions need to happen
            for(int i = 0; i < particleCollisionEventsToCome.Count; i++)
            {
                if (particleCollisionEventsToCome[i].timeOfCollision <= time - moveTime)
                {
                    Collide(particleCollisionEventsToCome[i]);
                    particleCollisionEventsToCome.Remove(particleCollisionEventsToCome[i]);
                }
            }

            collisionCheckTime -= moveTime;
            if (collisionCheckTime <= 0)
            {
                CheckParticleCollisions();
                collisionCheckTime = 1f / collisionCalcFreq;
            }
        }

        public ref Particle GetParticle(string particleID)
        {
            for(int i = 0; i < numberOfParticles; i++)
            {
                if(particles[i].GetID() == particleID)
                {
                    return ref particles[i];
                }
            }

            throw (new Exception("Particle with ID: " + particleID + " does not exist."));
        }

        public void CheckExistingParticles(string checkID)
        {
            for(int i = 0; i < numberOfParticles; i++)
            {
                if(particles[i].GetID() == checkID)
                {
                    throw (new Exception("Particle with ID: " + checkID + " already exists."));
                }
            }
        }

        private void CheckParticleCollisions()
        {
            List<Line> courses = new List<Line>();
            List<string> particleIDs = new List<string>();
            for(int i = 0; i < numberOfParticles; i++)
            {
                courses.Add(Line.CalcLineBetweenPoints(particles[i].GetPosition(), particles[i].GetProjection(1 / collisionCalcFreq)));
                particleIDs.Add(particles[i].GetID());
            }

            for(int a = 0; a < courses.Count; a++)
            {
                for(int b = 0; b < courses.Count; b++)
                {
                    Vector intersect = Line.CalcIntersection(courses[a], courses[b]);
                    if(courses[a].IsInBounds(intersect) && courses[b].IsInBounds(intersect))
                    {
                        var tempPartPos = GetParticle(particleIDs[a]).GetPosition();
                        tempPartPos.Multiply(-1);
                        var tempIntersect = intersect;
                        tempIntersect.Add(tempPartPos);
                        float timeToCol = tempIntersect.GetMagnitude() / GetParticle(particleIDs[a]).GetVelocity().GetMagnitude();
                        particleCollisionEventsToCome.Add(new ParticleCollisionEvent(particleIDs[a], particleIDs[b], intersect, timeToCol + time));
                    }
                }
            }

            for(int a = 0; a < particleCollisionEventsToCome.Count; a++)
            {
                for(int b = 0; b < particleCollisionEventsToCome.Count; b++)
                {
                    if (a == b) { continue; }
                    if((particleCollisionEventsToCome[a].particleID1 == particleCollisionEventsToCome[b].particleID1 && particleCollisionEventsToCome[a].particleID2 == particleCollisionEventsToCome[b].particleID2) || (particleCollisionEventsToCome[a].particleID2 == particleCollisionEventsToCome[b].particleID1 && particleCollisionEventsToCome[a].particleID1 == particleCollisionEventsToCome[b].particleID2))
                    {
                        particleCollisionEventsToCome.RemoveAt(b);
                    }
                }
            }

            if(particleCollisionEventsToCome.Count > 0)
            {
                Console.WriteLine("Time of collision: " + particleCollisionEventsToCome[0].timeOfCollision);
            }
        }

        private void Collide(ref Particle a, ref Particle b, Vector positionOfCollision)
        {
            if(!a.DoesOvelap(positionOfCollision) || !b.DoesOvelap(positionOfCollision))
            {
                return;
            }

            var normal = b.GetPosition();
            normal.Multiply(-1);
            normal.Add(a.GetPosition());
            normal.Normalise();

            var relativeV = b.GetVelocity();
            relativeV.Multiply(-1);
            relativeV.Add(a.GetVelocity());

            var elasticity = a.springK * b.springK;

            var impulseMag = -(1 + elasticity) * Vector.DotProduct(relativeV, normal);
            var tempN = normal;
            tempN.Multiply((1 / a.GetMass()) + (1 / b.GetMass()));
            impulseMag = impulseMag / Vector.DotProduct(Vector.Multiply(normal,(1 / a.GetMass()) + (1 / b.GetMass())), normal);

            var impulse = Vector.Multiply(normal, impulseMag);
            a.ApplyImpulse(impulse);
            b.ApplyImpulse(Vector.Multiply(impulse, -1));
        }

        private void Collide(ParticleCollisionEvent particleEvent)
        {
            Collide(ref GetParticle(particleEvent.particleID1), ref GetParticle(particleEvent.particleID2), particleEvent.positionOfCollision);
        }
    }
}
