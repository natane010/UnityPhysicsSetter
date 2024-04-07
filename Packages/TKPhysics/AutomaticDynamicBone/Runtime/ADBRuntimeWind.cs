using UnityEngine;

namespace TKPhysics
{
    public class ADBRuntimeWind
    {
        float accel;

        Vector3 getWindA()
        {
            return new Vector3(Mathf.PerlinNoise(Time.time, 0.0f) * 0.005f, 0, 0);

        }
        Vector3 getWindB()
        {
            accel += Time.deltaTime;
            return Vector3.left* (Mathf.Sin(accel) * 0.5f + 0.5f);
        }

    }

}