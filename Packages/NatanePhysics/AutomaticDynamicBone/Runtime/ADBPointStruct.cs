using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using System;
namespace TKPhysics
{
    [Serializable]
    public struct PointRead
    {
        public int fixedIndex;
        public int parentIndex;
        public int childFirstIndex;
        public int childLastIndex;
        public float mass;

        public int colliderMask;
        public bool isFixedPointFreezeRotation;
        public float damping;

        public float friction;
        public float stiffnessWorld;
        public float stiffnessLocal;
        public float elasticity;
        public float moveInert;
        public float velocityIncrease;
        public float3 gravity;

        public float structuralShrinkVertical;
        public float structuralStretchVertical;
        public float structuralShrinkHorizontal;
        public float structuralStretchHorizontal;
        public float shearShrink;
        public float shearStretch;
        public float bendingShrinkVertical;
        public float bendingStretchVertical;
        public float bendingShrinkHorizontal;
        public float bendingStretchHorizontal;
        public float circumferenceShrink;
        public float circumferenceStretch;
        public float radius;

        public float3 initialLocalPosition;
        internal float initialLocalPositionLength;

        public float3 initialPosition;

        internal quaternion initialLocalRotation;
        internal quaternion initialRotation;

        internal float dampDivIteration;
        internal float addForceScale;
        public float lengthLimitForceScale;
        public float elasticityVelocity;

    }

    public struct PointReadWrite
    {
        public float3 position;
        public quaternion rotationNoSelfRotateChange;
        public float3 deltaPosition;

        public quaternion deltaRotation;

        public bool isCollide;
    }
}
