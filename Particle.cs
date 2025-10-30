using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace _2D_Physics_Simulation
{
    public class Particle
    {
        private string id;

        private Vector position;
        private Vector velocity;
        private Vector resultantForce;

        private float size = 1;//ik particles should not have size, but idc
        private float mass;
        public float springK = 0.9f;

        public Color color;

        public Vector GetPosition() { return position; }
        public Color GetColor() { return color; }
        public string GetID() { return id; }
        public Vector GetVelocity() { return velocity; }
        public float GetMass() { return mass; }

        public Particle(string id, float mass, Vector position)
        {
            this.id = id;
            this.mass = mass;
            this.position = position;
            velocity = new Vector(0, 0);
            resultantForce = new Vector(0, 0);

            color = Color.White;
        }

        public void AddForce(Vector force)
        {
            resultantForce.Add(force);
        }

        public void ApplyForce(Vector force, float time)
        {
            force.Multiply(1 / mass);
            force.Multiply(time);
            velocity.Add(force);
        }

        public void ApplyResultantForce(float time)
        {
            resultantForce.Multiply(1 / mass);
            resultantForce.Multiply(time);
            velocity.Add(resultantForce);
            resultantForce.SetX(0);
            resultantForce.SetY(0);
        }

        public void ApplyImpulse(Vector impulse)
        {
            impulse.Multiply(1 / mass);
            velocity.Add(impulse);
        }

        public void UpdatePosition(float time)
        {
            position.Add(Vector.Multiply(velocity, time));
        }

        public Vector GetProjection(float forwardTime)
        {
            return Vector.Sum(position, Vector.Multiply(velocity, forwardTime));
        }

        public bool DoesOvelap(Vector pos)
        {
            pos.Multiply(-1);
            if(Vector.Sum(pos, position).GetMagnitude() <= size)
            {
                return true;
            }
            return false;
        }
    }
}
