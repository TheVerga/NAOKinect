using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NAOKinect
{
    class Vecto3Float
    {
        private float x;

        public Vecto3Float(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vecto3Float()
        {
            x = 0;
            y = 0;
            z = 0;
        }

        public float X
        {
            get { return x; }
            set { x = value; }
        }
        private float y;

        public float Y
        {
            get { return y; }
            set { y = value; }
        }
        private float z;

        public float Z
        {
            get { return z; }
            set { z = value; }
        }


        public float dot(Vecto3Float vect)
        {
            return this.x * vect.X + this.y * vect.Y + this.z * vect.Z;
        }

        public float magnitude()
        {
            return (float) Math.Sqrt((double )(x*x + y*y + z*z));
        }

        public void normalize()
        {
            float magnitude = this.magnitude();
            this.x = this.x / magnitude;
            this.y = this.y / magnitude;
            this.z = this.z / magnitude;
        }

        public Vecto3Float cross(Vecto3Float vec)
        {
            Vecto3Float res = new Vecto3Float();

            res.X = this.Y * vec.Z - vec.Y * this.Z;
            res.Y = this.Z * vec.X - vec.Z * this.X;
            res.Z = this.X * vec.Y - vec.Z * this.Y;

            return res;
        }

        public Vecto3Float projectionOntoXY()
        {
            return new Vecto3Float(this.X, this.Y, 0);
        }

        public Vecto3Float projectionOntoZX()
        {
            return new Vecto3Float(this.X, 0, this.Z);
        }

        public Vecto3Float projectionOntoZY()
        {
            return new Vecto3Float(0, this.y, this.z);
        }

        public Vecto3Float negate()
        {
            return new Vecto3Float(-this.x,- this.y, -this.z);
        }

    }
}
