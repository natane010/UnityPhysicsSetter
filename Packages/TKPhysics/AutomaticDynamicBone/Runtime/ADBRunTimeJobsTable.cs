using UnityEngine;
using UnityEngine.Jobs;
using Unity.Jobs;
using Unity.Jobs.LowLevel;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Burst;
using Unity.Mathematics;
using System.Runtime.CompilerServices;

namespace TKPhysics.Internal
{
    public static unsafe class ADBRunTimeJobsTable
    {
        #region Jobs
        [BurstCompile]
        public struct InitiralizePoint1 : IJobParallelForTransform //OYM:先更新fixed节点

        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public PointRead* pReadPoints;
            [NativeDisableUnsafePtrRestriction]
            public PointReadWrite* pReadWritePoints;
            internal float worldScale;

            public void Execute(int index, TransformAccess transform)
            {
                var pReadWritePoint = pReadWritePoints + index;
                var pReadPoint = pReadPoints + index;

                if (pReadPoint->parentIndex == -1)
                {
                    transform.localRotation = pReadPoint->initialLocalRotation;

                    pReadWritePoint->rotationNoSelfRotateChange = transform.rotation;
                    pReadWritePoint->position = transform.position / worldScale;

                    pReadWritePoint->deltaPosition = float3.zero;

                }
            }
        }
        [BurstCompile]
        public struct InitiralizePoint2 : IJobParallelForTransform 
        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public PointRead* pReadPoints;
            [NativeDisableUnsafePtrRestriction]
            public PointReadWrite* pReadWritePoints;
            internal float worldScale;

            public void Execute(int index, TransformAccess transform)
            {
                var pReadWritePoint = pReadWritePoints + index;
                var pReadPoint = pReadPoints + index;

                if (pReadPoint->parentIndex != -1)
                {
                    var pFixReadWritePoint = pReadWritePoints + (pReadPoint->fixedIndex);
                    var pFixReadPoint = pReadPoints + (pReadPoint->fixedIndex);
                    transform.localRotation = pReadPoint->initialLocalRotation;
                    float3 transformPosition = (pFixReadWritePoint->position + math.mul(pFixReadWritePoint->rotationNoSelfRotateChange, pReadPoint->initialPosition));

                    pReadWritePoint->position = transformPosition;
                    transform.position = transformPosition * worldScale;
                    pReadWritePoint->deltaPosition = float3.zero;

                }
            }
        }

        [BurstCompile]
        public struct ColliderClacAABB : IJobParallelFor
        {
            [NativeDisableUnsafePtrRestriction]
            public ColliderRead* pReadColliders;
            [NativeDisableUnsafePtrRestriction]
            public ColliderReadWrite* pReadWriteColliders;
            [ReadOnly]
            public float oneDivideIteration;
            [ReadOnly]
            public float localScale;
            [ReadOnly]
            public float maxPointRadius;
            public void Execute(int index)
            {

                ColliderRead* pReadCollider = pReadColliders + index;

                ColliderReadWrite* pReadWriteCollider = pReadWriteColliders + index;
                pReadWriteCollider->collideFunc = pReadCollider->collideFunc;
                pReadWriteCollider->colliderType = pReadCollider->colliderType;
                float colliderScale = math.cmax(pReadCollider->scale) * localScale;
                float3 fromLocalPosition = pReadCollider->fromPosition * localScale;
                float3 toLocalPosition = pReadCollider->toPosition * localScale;

                pReadWriteCollider->position = pReadCollider->fromPosition * localScale;
                pReadCollider->deltaPosition = (pReadCollider->toPosition - pReadCollider->fromPosition) * localScale * oneDivideIteration;
                MinMaxAABB AABB, temp1, temp2;
                switch (pReadCollider->colliderType)
                {
                    case ColliderType.Sphere:

                        pReadWriteCollider->size = new float3(pReadCollider->originRadius * colliderScale, 0, 0);


                        AABB = new MinMaxAABB(fromLocalPosition, toLocalPosition);
                        AABB.Expand(pReadCollider->originRadius * colliderScale);


                        break;
                    case ColliderType.Capsule:

                        pReadWriteCollider->direction = pReadCollider->fromDirection;
                        pReadCollider->deltaDirection = (pReadCollider->toDirection - pReadCollider->fromDirection) * oneDivideIteration;
                        pReadWriteCollider->size = new float3(pReadCollider->originRadius * colliderScale, pReadCollider->originHeight * colliderScale, 0);


                        temp1 = new MinMaxAABB(fromLocalPosition, fromLocalPosition + pReadCollider->fromDirection * pReadCollider->originHeight * colliderScale); 
                        temp2 = new MinMaxAABB(toLocalPosition, toLocalPosition + pReadCollider->toDirection * pReadCollider->originHeight * colliderScale); 
                        AABB = new MinMaxAABB(temp1, temp2);
                        AABB.Expand(pReadCollider->originRadius * colliderScale);

                        break;
                    case ColliderType.OBB:

                        pReadWriteCollider->rotation = pReadCollider->fromRotation;
                        pReadCollider->deltaRotation = math.slerp(pReadCollider->fromRotation, pReadCollider->toRotation, oneDivideIteration);
                        pReadWriteCollider->size = pReadCollider->originBoxSize * pReadCollider->scale * localScale;

                        temp1 = MinMaxAABB.CreateFromCenterAndHalfExtents(fromLocalPosition, pReadCollider->originBoxSize * colliderScale); 
                        temp1 = MinMaxAABB.Rotate(pReadCollider->fromRotation, temp1);
                        temp2 = MinMaxAABB.CreateFromCenterAndHalfExtents(toLocalPosition, pReadCollider->originBoxSize * colliderScale);
                        temp2 = MinMaxAABB.Rotate(pReadCollider->toRotation, temp2);
                        AABB = new MinMaxAABB(temp1, temp2);
                        break;
                    default:
                        AABB = MinMaxAABB.identity;
                        break;
                }
                pReadCollider->AABB = AABB;
            }
        }


        [BurstCompile]
        public struct PointGetTransform : IJobParallelForTransform
        {
            [NativeDisableUnsafePtrRestriction]
            public PointRead* pReadPoints;
            [NativeDisableUnsafePtrRestriction]
            public PointReadWrite* pReadWritePoints;
            [ReadOnly]
            public float oneDivideIteration;
            [ReadOnly]
            public float worldScale;
            public void Execute(int index, TransformAccess transform)
            {
                PointRead* pReadPoint = pReadPoints + index;
                PointReadWrite* pReadWritePoint = pReadWritePoints + index;

                quaternion transformRotation = transform.rotation;
                float3 transformPosition = transform.position / worldScale;
                quaternion localRotation = transform.localRotation;
                quaternion parentRotation = math.mul(transformRotation, math.inverse(localRotation));
                quaternion currentRotationNoSelfRotateChange = math.mul(parentRotation, pReadPoint->initialLocalRotation);
                if (pReadPoint->parentIndex == -1)
                {
                    pReadWritePoint->deltaPosition = (transformPosition - pReadWritePoint->position);
                    pReadWritePoint->deltaRotation = math.slerp(pReadWritePoint->rotationNoSelfRotateChange, currentRotationNoSelfRotateChange, oneDivideIteration);
                    pReadWritePoint->rotationNoSelfRotateChange = currentRotationNoSelfRotateChange;

                }
                else
                {
                    pReadPoint->dampDivIteration = math.exp(math.log(pReadPoint->damping) * oneDivideIteration);
                    pReadWritePoint->rotationNoSelfRotateChange = currentRotationNoSelfRotateChange;
                }

            }
        }

        [BurstCompile]
        public struct PointUpdate : IJobParallelFor
        {
            const float gravityLimit = 1f;
            [NativeDisableUnsafePtrRestriction]
            internal PointReadWrite* pReadWritePoints;
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            internal PointRead* pReadPoints;
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public ColliderRead* pReadColliders;
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public ColliderReadWrite* pReadWriteColliders;
            [ReadOnly]
            public int colliderCount;
            [ReadOnly]
            internal float3 addForcePower;
            [ReadOnly]
            internal float oneDivideIteration;
            [ReadOnly]
            internal float deltaTime;
            [ReadOnly]
            internal bool isCollision;
            [ReadOnly]
            internal bool isOptimize;
            public void Execute(int index)
            {
                PointRead* pReadPoint = pReadPoints + index;
                PointReadWrite* pReadWritePoint = pReadWritePoints + index;
                if (pReadPoint->fixedIndex != index)
                {

                    EvaluatePosition(index, pReadPoint, pReadWritePoint, addForcePower, oneDivideIteration, deltaTime, isOptimize);
                    if (isCollision)
                    {
                        for (int i = 0; i < colliderCount; ++i)
                        {
                            ColliderRead* pReadCollider = pReadColliders + i;

                            if (pReadCollider->isOpen && (pReadPoint->colliderMask & pReadCollider->colliderChoice) != 0)
                            {
                                float pointRadius = pReadPoint->radius;
                                bool isColliderInsideMode = (pReadCollider->collideFunc == CollideFunc.InsideLimit || pReadCollider->collideFunc == CollideFunc.InsideNoLimit); 
                                if (pReadCollider->AABB.Overlaps(pReadWritePoint->position - pointRadius, pReadWritePoint->position + pointRadius) ^ isColliderInsideMode)
                                {
                                    ColliderReadWrite* pReadWriteCollider = pReadWriteColliders + i;
                                    CollideProcess(pReadPoint, pReadWritePoint, pReadWriteCollider, pointRadius, oneDivideIteration, isColliderInsideMode);
                                }
                            }
                        }
                    }
                }
                pReadWritePoint->position += oneDivideIteration * pReadWritePoint->deltaPosition * deltaTime * 60;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void EvaluatePosition(int index, PointRead* pReadPointTarget, PointReadWrite* pReadWritePointTarget, float3 addForcePower, float oneDivideIteration, float deltaTime, bool isOptimize)
            {
                float timeScale = deltaTime * 60;
                pReadWritePointTarget->deltaPosition *= pReadPointTarget->dampDivIteration;

                if (pReadPointTarget->stiffnessLocal != 0 || pReadPointTarget->elasticity != 0 || pReadPointTarget->elasticityVelocity != 0 || pReadPointTarget->lengthLimitForceScale != 0)
                {
                    UpdateDynamicBone(index, pReadPointTarget, pReadWritePointTarget, oneDivideIteration, timeScale);
                }
                if (pReadPointTarget->velocityIncrease != 0 || pReadPointTarget->moveInert != 0)
                {
                    UpdateFixedPointChain(index, pReadPointTarget, pReadWritePointTarget, oneDivideIteration);
                }

                if (math.any(pReadPointTarget->gravity))
                {
                    UpdateGravity(pReadPointTarget, pReadWritePointTarget, deltaTime, oneDivideIteration);
                }

                if (pReadPointTarget->stiffnessWorld != 0)
                {
                    UpdateFreeze(index, pReadPointTarget, pReadWritePointTarget, oneDivideIteration, deltaTime);
                }

                if (math.any(addForcePower))
                {
                    UpdateExternalForce(pReadPointTarget, pReadWritePointTarget, addForcePower, oneDivideIteration); 
                }


                if (isOptimize)
                {
                    OptimeizeForce(pReadPointTarget, pReadWritePointTarget); 
                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateDynamicBone(int index, PointRead* pReadPointTarget, PointReadWrite* pReadWritePointTarget, float oneDivideIteration, float timeScale)
            {

                PointReadWrite* pPointReadWriteParent = pReadWritePointTarget + (pReadPointTarget->parentIndex - index);

                float3 targetDirection = math.mul(pPointReadWriteParent->rotationNoSelfRotateChange, pReadPointTarget->initialLocalPosition) * pReadPointTarget->initialLocalPositionLength;

                float3 currentDirection = pReadWritePointTarget->position - pPointReadWriteParent->position;

                float3 difficult = currentDirection - targetDirection;

                float difficultLength = math.max(math.EPSILON, math.length(difficult));

                float stiffnessLength = pReadPointTarget->initialLocalPositionLength * 2 * (1 - pReadPointTarget->stiffnessLocal);

                float stiffnessForceLength = math.clamp(difficultLength, 0, stiffnessLength) - difficultLength;

                currentDirection += difficult / difficultLength * stiffnessForceLength;

                float3 lerpDirection = math.lerp(currentDirection, targetDirection, pReadPointTarget->elasticity); 

                float lerpDirectionLength = math.max(math.EPSILON, math.length(lerpDirection));

                lerpDirection *= math.lerp(lerpDirectionLength, pReadPointTarget->initialLocalPositionLength, pReadPointTarget->lengthLimitForceScale) / lerpDirectionLength; 

                float3 move = (pPointReadWriteParent->position + lerpDirection - pReadWritePointTarget->position) * math.min(0.5f, oneDivideIteration * timeScale);

                pReadWritePointTarget->position += move;
                pReadWritePointTarget->deltaPosition += move * pReadPointTarget->elasticityVelocity * oneDivideIteration;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateFixedPointChain(int index, PointRead* pReadPointTarget, PointReadWrite* pReadWritePointTarget, float oneDivideIteration)
            {
                PointReadWrite* pPointReadWriteFixed = pReadWritePointTarget + (pReadPointTarget->fixedIndex - index);
                PointRead* pPointReadFixed = pReadPointTarget + (pReadPointTarget->fixedIndex - index);
                float3 fixedPointdeltaPosition = pPointReadWriteFixed->deltaPosition;
                pReadWritePointTarget->position += fixedPointdeltaPosition * pReadPointTarget->moveInert * oneDivideIteration;
                pReadWritePointTarget->deltaPosition -= fixedPointdeltaPosition * pReadPointTarget->velocityIncrease * 0.2f * oneDivideIteration;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateGravity(PointRead* pReadPointTarget, PointReadWrite* pReadWritePointTarget, float deltaTime, float oneDivideIteration)
            {
                {
                    float3 gravity = pReadPointTarget->gravity * (deltaTime * deltaTime);

                    pReadWritePointTarget->deltaPosition += gravity * oneDivideIteration;
                }
                

            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateFreeze(int index, PointRead* pReadPointTarget, PointReadWrite* pReadWritePointTarget, float oneDivideIteration, float deltatime)
            {
                PointReadWrite* pPointReadWriteFixed = pReadWritePointTarget + (pReadPointTarget->fixedIndex - index);
                PointRead* pPointReadFixed = pReadPointTarget + (pReadPointTarget->fixedIndex - index);

                float3 fixedPointPosition = pPointReadWriteFixed->position;
                float3 direction = pReadWritePointTarget->position - fixedPointPosition;

                quaternion fixedPointRotation = pPointReadWriteFixed->rotationNoSelfRotateChange;
                float3 originDirection = math.mul(fixedPointRotation, pReadPointTarget->initialPosition);

                float3 freezeForce = originDirection - direction;

                float freezeForceLength = math.max(math.EPSILON, math.length(freezeForce));
                freezeForceLength = math.sqrt(freezeForceLength);

                float freezeForcelengthLimit = math.clamp(freezeForceLength, -pReadPointTarget->stiffnessWorld * 0.1f, pReadPointTarget->stiffnessWorld * 0.1f);
                freezeForce *= (freezeForcelengthLimit / freezeForceLength);
                freezeForce = oneDivideIteration * deltatime * pReadPointTarget->stiffnessWorld * freezeForce;
                pReadWritePointTarget->deltaPosition += freezeForce;


            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateExternalForce(PointRead* pReadPointTarget, PointReadWrite* pReadWritePointTarget, float3 addForcePower, float oneDivideIteration)
            {
                float3 addForce = oneDivideIteration * addForcePower * pReadPointTarget->addForceScale / pReadPointTarget->mass;
                pReadWritePointTarget->deltaPosition += addForce;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void OptimeizeForce(PointRead* pReadPointTarget, PointReadWrite* pReadWritePointTarget)
            {
                float persentage;

                persentage = math.max(math.EPSILON, math.length(pReadWritePointTarget->deltaPosition) / pReadPointTarget->initialLocalPositionLength);
                pReadWritePointTarget->deltaPosition *= math.min(1, persentage) / persentage;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private void CollideProcess(PointRead* pReadPoint, PointReadWrite* pReadWritePoint, ColliderReadWrite* pReadWriteCollider, float pointRadius, float oneDivideIteration, bool isColliderInsideMode)
            {
                float3 colliderPosition = pReadWriteCollider->position;
                float3 size = pReadWriteCollider->size;
                float3 pushout;
                float radiusSum;

                switch (pReadWriteCollider->colliderType)
                {
                    case ColliderType.Sphere: 
                        radiusSum = size.x + pointRadius;
                        pushout = pReadWritePoint->position - colliderPosition;
                        ClacPowerWhenCollision(pushout, radiusSum, pReadPoint, pReadWritePoint, pReadWriteCollider->collideFunc, oneDivideIteration);

                        break;

                    case ColliderType.Capsule: 

                        float3 colliderDirection = pReadWriteCollider->direction;
                        radiusSum = pointRadius + size.x;
                        pushout = pReadWritePoint->position - ConstrainToSegment(pReadWritePoint->position, colliderPosition, colliderDirection * size.y);
                        ClacPowerWhenCollision(pushout, radiusSum, pReadPoint, pReadWritePoint, pReadWriteCollider->collideFunc, oneDivideIteration);

                        break;
                    case ColliderType.OBB: //OYM:OBB

                        quaternion colliderRotation = pReadWriteCollider->rotation;
                        var localPosition = math.mul(math.inverse(colliderRotation), (pReadWritePoint->position - colliderPosition)); 
                        MinMaxAABB localOBB = MinMaxAABB.CreateFromCenterAndHalfExtents(0, size + pointRadius);

                        if (localOBB.Contains(localPosition) ^ isColliderInsideMode)
                        {
                            if (isColliderInsideMode)
                            {
                                pushout = math.clamp(localPosition, localOBB.Min, localOBB.Max) - localPosition;
                            }
                            else
                            {
                                float3 toMax = localOBB.Max - localPosition;
                                float3 toMin = localOBB.Min - localPosition;
                                float3 min3 = new float3
                                (
                                math.abs(toMax.x) < math.abs(toMin.x) ? toMax.x : toMin.x,
                                math.abs(toMax.y) < math.abs(toMin.y) ? toMax.y : toMin.y,
                                math.abs(toMax.z) < math.abs(toMin.z) ? toMax.z : toMin.z
                                );
                                float3 min3Abs = math.abs(min3);
                                if (min3Abs.x <= min3Abs.y && min3Abs.x <= min3Abs.z)
                                {
                                    pushout = new float3(min3.x, 0, 0);
                                }
                                else if (min3Abs.y <= min3Abs.x && min3Abs.y <= min3Abs.z)
                                {
                                    pushout = new float3(0, min3.y, 0);
                                }
                                else
                                {
                                    pushout = new float3(0, 0, min3.z);
                                }
                            }

                            pushout = math.mul(colliderRotation, pushout);

                            DistributionPower(pushout, pReadPoint, pReadWritePoint, pReadWriteCollider->collideFunc, oneDivideIteration);

                        }
                        break;
                    default:
                        return;
                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void ClacPowerWhenCollision(float3 pushout, float radius, PointRead* pReadPoint, PointReadWrite* pReadWritePoint, CollideFunc collideFunc, float oneDivideIteration)
            {
                float sqrPushout = math.lengthsq(pushout);
                switch (collideFunc)
                {
                    case CollideFunc.OutsideLimit:
                        if ((sqrPushout > radius * radius) && sqrPushout != 0) 
                        { return; }
                        break;
                    case CollideFunc.InsideLimit:
                        if (sqrPushout < radius * radius && sqrPushout != 0)
                        { return; }
                        break;
                    case CollideFunc.OutsideNoLimit:
                        if ((sqrPushout > radius * radius) && sqrPushout != 0)
                        { return; }
                        break;
                    case CollideFunc.InsideNoLimit:
                        if (sqrPushout < radius * radius && sqrPushout != 0)
                        { return; }
                        break;
                    default: { return; }

                }

                pushout = pushout * (radius / math.sqrt(sqrPushout) - 1);
                DistributionPower(pushout, pReadPoint, pReadWritePoint, collideFunc, oneDivideIteration);

            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void DistributionPower(float3 pushout, PointRead* pReadPoint, PointReadWrite* pReadWritePoint, CollideFunc collideFunc, float oneDivideIteration)
            {
                float sqrPushout = math.lengthsq(pushout);
                if (collideFunc == CollideFunc.InsideNoLimit || collideFunc == CollideFunc.OutsideNoLimit)
                {
                    pReadWritePoint->deltaPosition += 0.01f * oneDivideIteration * pReadPoint->addForceScale * pushout;
                }
                else
                {

                    pReadWritePoint->deltaPosition += pushout;
                    pReadWritePoint->deltaPosition *= (1 - pReadPoint->friction);
                    pReadWritePoint->position += pushout;

                }


            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static float3 ConstrainToSegment(float3 tag, float3 pos, float3 dir)
            {
                float t = math.dot(tag - pos, dir) / math.lengthsq(dir);
                return pos + dir * math.clamp(t, 0, 1);
            }
        }

        [BurstCompile]
        public struct ColliderPositionUpdate : IJobParallelFor
        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public ColliderRead* pReadColliders;
            [NativeDisableUnsafePtrRestriction]
            public ColliderReadWrite* pReadWriteColliders;
            [ReadOnly]
            public float oneDivideIteration;
            public void Execute(int index)
            {
                ColliderReadWrite* pReadWriteCollider = pReadWriteColliders + index;
                ColliderRead* pReadCollider = pReadColliders + index;

                switch (pReadCollider->colliderType)
                {
                    case ColliderType.Sphere:
                        pReadWriteCollider->position += pReadCollider->deltaPosition;
                        break;
                    case ColliderType.Capsule:
                        pReadWriteCollider->position += pReadCollider->deltaPosition;
                        pReadWriteCollider->direction += pReadCollider->deltaDirection;
                        break;
                    case ColliderType.OBB:
                        pReadWriteCollider->position += pReadCollider->deltaPosition;
                        pReadWriteCollider->rotation = math.mul(pReadCollider->deltaRotation, pReadWriteCollider->rotation);
                        break;
                    default:
                        break;

                }
            }
        }

        [BurstCompile]
        public struct ConstraintUpdate : IJobParallelFor
        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public PointRead* pReadPoints;
            [NativeDisableUnsafePtrRestriction]
            public PointReadWrite* pReadWritePoints;
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public ColliderRead* pReadColliders;
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public ColliderReadWrite* pReadWriteColliders;
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public ConstraintRead* pConstraintsRead;
            [ReadOnly]
            public int colliderCount;
            [ReadOnly]
            public int globalColliderCount;
            [ReadOnly]
            public bool isCollision;
            [ReadOnly]
            internal float oneDivideIteration;

            public void Execute(int index)
            {
                ConstraintRead* constraint = pConstraintsRead + index;

                PointRead* pPointReadA = pReadPoints + constraint->indexA;
                PointRead* pPointReadB = pReadPoints + constraint->indexB;
                if (pPointReadA->parentIndex == -1 && pPointReadB->parentIndex == -1)
                { return; }
                PointReadWrite* pReadWritePointA = pReadWritePoints + constraint->indexA;

                PointReadWrite* pReadWritePointB = pReadWritePoints + constraint->indexB;
                float3 positionA = pReadWritePointA->position;
                float3 positionB = pReadWritePointB->position;

                float WeightProportion = pPointReadB->mass / (pPointReadA->mass + pPointReadB->mass);

                var Direction = positionB - positionA;
                if (math.all(Direction == 0))
                {
                    return;
                }
                float Distance = math.length(Direction);


                float originDistance = constraint->length;
                float Force = Distance - math.clamp(Distance, originDistance * constraint->shrink, originDistance * constraint->stretch);
                if (Force != 0)
                {
                    bool IsShrink = Force >= 0.0f;
                    float ConstraintPower;
                    switch (constraint->type)
                    {
                        case ConstraintType.Structural_Vertical:
                            ConstraintPower = IsShrink
                                ? (pPointReadA->structuralShrinkVertical + pPointReadB->structuralShrinkVertical)
                                : (pPointReadA->structuralStretchVertical + pPointReadB->structuralStretchVertical);
                            break;
                        case ConstraintType.Structural_Horizontal:
                            ConstraintPower = IsShrink
                                ? (pPointReadA->structuralShrinkHorizontal + pPointReadB->structuralShrinkHorizontal)
                                : (pPointReadA->structuralStretchHorizontal + pPointReadB->structuralStretchHorizontal);
                            break;
                        case ConstraintType.Shear:
                            ConstraintPower = IsShrink
                                ? (pPointReadA->shearShrink + pPointReadB->shearShrink)
                                : (pPointReadA->shearStretch + pPointReadB->shearStretch);
                            break;
                        case ConstraintType.Bending_Vertical:
                            ConstraintPower = IsShrink
                                ? (pPointReadA->bendingShrinkVertical + pPointReadB->bendingShrinkVertical)
                                : (pPointReadA->bendingStretchVertical + pPointReadB->bendingStretchVertical);
                            break;
                        case ConstraintType.Bending_Horizontal:
                            ConstraintPower = IsShrink
                                ? (pPointReadA->bendingShrinkHorizontal + pPointReadB->bendingShrinkHorizontal)
                                : (pPointReadA->bendingStretchHorizontal + pPointReadB->bendingStretchHorizontal);
                            break;
                        case ConstraintType.Circumference:
                            ConstraintPower = IsShrink
                                ? (pPointReadA->circumferenceShrink + pPointReadB->circumferenceShrink)
                                : (pPointReadA->circumferenceStretch + pPointReadB->circumferenceStretch);
                            break;
                        default:
                            ConstraintPower = 0.0f;
                            break;
                    }


                    if (ConstraintPower > 0.0f)
                    {
                        float3 Displacement = Direction / Distance * (Force * ConstraintPower);

                        pReadWritePointA->position += Displacement * WeightProportion;
                        pReadWritePointA->deltaPosition += Displacement * WeightProportion;
                        pReadWritePointB->position += -Displacement * (1 - WeightProportion);
                        pReadWritePointB->deltaPosition += -Displacement * (1 - WeightProportion);
                    }

                }
                if (isCollision && constraint->isCollider)
                {
                    for (int i = 0; i < colliderCount; ++i)
                    {
                        ColliderRead* pReadCollider = pReadColliders + i;

                        if (!(pReadCollider->isOpen && (pPointReadA->colliderMask & pReadCollider->colliderChoice) != 0))
                        { continue; }

                        MinMaxAABB constraintAABB = new MinMaxAABB(positionA, positionB);
                        constraintAABB.Expand(constraint->radius);
                        bool isColliderInsideMode = (pReadCollider->collideFunc == CollideFunc.InsideLimit || pReadCollider->collideFunc == CollideFunc.InsideNoLimit); 

                        if (pReadCollider->AABB.Overlaps(constraintAABB) ^ isColliderInsideMode)
                        {
                            ColliderReadWrite* pReadWriteCollider = pReadWriteColliders + i;
                            ComputeCollider(
                                pReadWriteCollider,
                                pPointReadA, pPointReadB,
                                pReadWritePointA, pReadWritePointB, positionA, positionB,
                                constraint,
                                WeightProportion, oneDivideIteration, isColliderInsideMode
                                );
                        }


                    }
                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void ComputeCollider(ColliderReadWrite* pReadColliderReadWrite,
                PointRead* pReadPointA, PointRead* pReadPointB,
                PointReadWrite* pReadWritePointA, PointReadWrite* pReadWritePointB,
                float3 positionA, float3 positionB,
                ConstraintRead* constraint,
                float WeightProportion, float oneDivideIteration, bool isColliderInsideMode)
            {
                float throwTemp;
                float t, radius;
                float3 size = pReadColliderReadWrite->size;

                float3 colliderPosition = pReadColliderReadWrite->position;

                switch (pReadColliderReadWrite->colliderType)
                {
                    case ColliderType.Sphere:
                        {
                            radius = size.x + constraint->radius;

                            {
                                float3 pointOnLine = ConstrainToSegment(colliderPosition, positionA, positionB - positionA, out t);
                                ClacPowerWhenCollision(pointOnLine - colliderPosition, radius,
                                    pReadPointA, pReadPointB, pReadWritePointA, pReadWritePointB,
                                    WeightProportion, t, oneDivideIteration,
                                    pReadColliderReadWrite->collideFunc);
                            }
                        }

                        break;
                    case ColliderType.Capsule:
                        {
                            radius = size.x + constraint->radius;
                            float3 colliderDirection = pReadColliderReadWrite->direction;

                            {
                                float3 pointOnCollider, pointOnLine;
                                SqrComputeNearestPoints(colliderPosition, colliderDirection * size.y, positionA, positionB - positionA, out throwTemp, out t, out pointOnCollider, out pointOnLine);
                                ClacPowerWhenCollision(pointOnLine - pointOnCollider, radius,
                                    pReadPointA, pReadPointB, pReadWritePointA, pReadWritePointB,
                                    WeightProportion, t, oneDivideIteration,
                                    pReadColliderReadWrite->collideFunc);
                            }
                        }

                        break;
                    case ColliderType.OBB:
                        {
                            quaternion colliderRotation = pReadColliderReadWrite->rotation;
                            float3 boxSize = size + new float3(constraint->radius);

                            float t1, t2;
                            SegmentToOBB(positionA, positionB, colliderPosition, boxSize, math.inverse(colliderRotation), out t1, out t2);

                            t1 = math.saturate(t1);
                            t2 = math.saturate(t2);
                            bool bHit = t1 >= 0f && t2 > t1 && t2 <= 1.0f;

                            if (bHit && !isColliderInsideMode) 
                            {
                                float3 pushout;
                                t = (t1 + t2) * 0.5f;
                                float3 dir = positionB - positionA;
                                float3 nearestPoint = positionA + dir * t;
                                pushout = math.mul(math.inverse(colliderRotation), (nearestPoint - colliderPosition));
                                float pushoutX = pushout.x > 0 ? boxSize.x - pushout.x : -boxSize.x - pushout.x;
                                float pushoutY = pushout.y > 0 ? boxSize.y - pushout.y : -boxSize.y - pushout.y;
                                float pushoutZ = pushout.z > 0 ? boxSize.z - pushout.z : -boxSize.z - pushout.z;
                                if (math.abs(pushoutZ) <= math.abs(pushoutY) && math.abs(pushoutZ) <= math.abs(pushoutX))
                                {
                                    pushout = math.mul(colliderRotation, new float3(0, 0, pushoutZ));

                                }
                                else if (math.abs(pushoutY) <= math.abs(pushoutX) && math.abs(pushoutY) <= math.abs(pushoutZ))
                                {
                                    pushout = math.mul(colliderRotation, new float3(0, pushoutY, 0));
                                }
                                else
                                {
                                    pushout = math.mul(colliderRotation, new float3(pushoutX, 0, 0));
                                }
                                DistributionPower(pushout,
                                pReadPointA, pReadPointB, pReadWritePointA, pReadWritePointB,
                                WeightProportion, t, oneDivideIteration,
                                pReadColliderReadWrite->collideFunc);

                            }
                            bool bOutside = t1 <= 0f || t1 >= 1f || t2 <= 0 || t2 >= 1f;
                            if (bOutside && isColliderInsideMode) 
                            {

                                float3 localPositionA = math.mul(math.inverse(colliderRotation), positionA - colliderPosition);
                                float3 localPositionB = math.mul(math.inverse(colliderRotation), positionB - colliderPosition);
                                float3 pushA = math.clamp(localPositionA, -boxSize, boxSize) - localPositionA;
                                float3 pushB = math.clamp(localPositionB, -boxSize, boxSize) - localPositionB;

                                pushA = math.mul(colliderRotation, pushA);
                                pushB = math.mul(colliderRotation, pushB);
                                bool isFixedA = WeightProportion < 1e-6f;
                                if (!isFixedA) 
                                {
                                    if (pReadColliderReadWrite->collideFunc == CollideFunc.InsideNoLimit)
                                    {
                                        pReadWritePointA->deltaPosition += 0.01f * oneDivideIteration * pReadPointA->addForceScale * pushA;
                                    }
                                    else
                                    {
                                        pReadWritePointA->deltaPosition += pushA;
                                        pReadWritePointA->deltaPosition *= (1 - pReadPointA->friction);

                                        positionA += pushA;

                                    }
                                }

                                if (pReadColliderReadWrite->collideFunc == CollideFunc.InsideNoLimit) 
                                {
                                    pReadWritePointB->deltaPosition += 0.01f * oneDivideIteration * pReadPointB->addForceScale * pushB;
                                }
                                else
                                {
                                    pReadWritePointB->deltaPosition += pushB;
                                    pReadWritePointB->deltaPosition *= (1 - pReadPointB->friction);

                                    positionB += pushB;

                                }

                            }

                            break;
                        }
                    default:
                        return;

                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void ClacPowerWhenCollision(float3 pushout, float radius,
                 PointRead* pReadPointA, PointRead* pReadPointB,
                PointReadWrite* pReadWritePointA, PointReadWrite* pReadWritePointB,
                float WeightProportion, float lengthPropotion, float oneDivideIteration,
                CollideFunc collideFunc)
            {
                float sqrPushout = math.lengthsq(pushout);
                switch (collideFunc)
                {
                    case CollideFunc.Freeze:
                        break;
                    case CollideFunc.OutsideLimit:
                        if (!(sqrPushout < radius * radius) && sqrPushout != 0)
                        { return; }
                        break;
                    case CollideFunc.InsideLimit:
                        if (sqrPushout < radius * radius && sqrPushout != 0)
                        { return; }
                        break;
                    case CollideFunc.OutsideNoLimit:
                        if (!(sqrPushout < radius * radius) && sqrPushout != 0)
                        { return; }
                        break;
                    case CollideFunc.InsideNoLimit:
                        if (sqrPushout < radius * radius && sqrPushout != 0)
                        { return; }
                        break;

                }
                pushout = pushout * (radius / Mathf.Sqrt(sqrPushout) - 1);
                DistributionPower(pushout,
                    pReadPointA, pReadPointB, pReadWritePointA, pReadWritePointB,
                    WeightProportion, lengthPropotion, oneDivideIteration,
                    collideFunc);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void DistributionPower(float3 pushout,
                 PointRead* pReadPointA, PointRead* pReadPointB, PointReadWrite* pReadWritePointA, PointReadWrite* pReadWritePointB,
                float WeightProportion, float lengthPropotion, float oneDivideIteration,
                CollideFunc collideFunc)
            {
                float sqrPushout = math.lengthsq(pushout);
                if (WeightProportion > 1e-6f)
                {
                    if (collideFunc == CollideFunc.InsideNoLimit || collideFunc == CollideFunc.OutsideNoLimit)
                    {
                        pReadWritePointA->deltaPosition += 0.01f * oneDivideIteration * (1 - lengthPropotion) * pReadPointA->addForceScale * pushout;
                    }
                    else
                    {
                        pReadWritePointA->deltaPosition += (pushout * (1 - lengthPropotion));
                        pReadWritePointA->deltaPosition *= (1 - pReadPointA->friction);

                        pReadWritePointA->position += (pushout * (1 - lengthPropotion));

                    }
                }
                else
                {
                    lengthPropotion = 1;
                }

                if (collideFunc == CollideFunc.InsideNoLimit || collideFunc == CollideFunc.OutsideNoLimit)
                {
                    pReadWritePointB->deltaPosition += 0.01f * oneDivideIteration * (lengthPropotion) * pReadPointB->addForceScale * pushout;
                }
                else
                {
                    pReadWritePointB->deltaPosition += (pushout * lengthPropotion);
                    pReadWritePointB->deltaPosition *= (1 - pReadPointB->friction);

                    pReadWritePointB->position += (pushout * lengthPropotion);

                }

            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static float SqrComputeNearestPoints(
                float3 posP,
                float3 dirP,
                float3 posQ,
                float3 dirQ,
out float tP, out float tQ, out float3 pointOnP, out float3 pointOnQ)
            {
                float lineDirSqrMag = math.lengthsq(dirQ);
                float3 inPlaneA = posP - ((math.dot(posP - posQ, dirQ) / lineDirSqrMag) * dirQ);
                float3 inPlaneB = posP + dirP - ((math.dot(posP + dirP - posQ, dirQ) / lineDirSqrMag) * dirQ);
                float3 inPlaneBA = inPlaneB - inPlaneA;

                float t1 = math.dot(posQ - inPlaneA, inPlaneBA) / math.lengthsq(inPlaneBA);
                t1 = math.all(inPlaneA != inPlaneB) ? t1 : 0f; 
                float3 L1ToL2Line = posP + dirP * math.saturate(t1);

                pointOnQ = ConstrainToSegment(L1ToL2Line, posQ, dirQ, out tQ);
                pointOnP = ConstrainToSegment(pointOnQ, posP, dirP, out tP);
                return math.lengthsq(pointOnP - pointOnQ);
            }

            static float3 ConstrainToSegment(float3 tag, float3 pos, float3 dir, out float t)
            {
                t = math.dot(tag - pos, dir) / math.lengthsq(dir);
                t = math.saturate(t);
                return pos + dir * t;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void SegmentToOBB(float3 start, float3 end, float3 center, float3 size, quaternion InverseNormal, out float t1, out float t2)
            {
                float3 startP = math.mul(InverseNormal, (center - start));
                float3 endP = math.mul(InverseNormal, (center - end));
                SegmentToAABB(startP, endP, center, -size, size, out t1, out t2);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void SegmentToAABB(float3 start, float3 end, float3 center, float3 min, float3 max, out float t1, out float t2)
            {
                float3 dir = end - start;
                t1 = math.cmax(math.min((min - start) / dir, (max - start) / dir));
                t2 = math.cmin(math.max((min - start) / dir, (max - start) / dir));
            }

        }

        [BurstCompile]
        public struct ConstraintForceUpdateByPoint : IJobParallelFor 
        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public PointRead* pReadPoints;
            [NativeDisableUnsafePtrRestriction]
            public PointReadWrite* pReadWritePoints;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public NativeParallelMultiHashMap<int, ConstraintRead> constraintsRead;
            [ReadOnly]
            internal float oneDivideIteration;
            public void Execute(int index)
            {
                PointRead* pPointReadA = pReadPoints + index;
                if (pPointReadA->parentIndex < 0)
                {
                    return;
                }

                NativeParallelMultiHashMapIterator<int> iterator;
                ConstraintRead constraint;
                float3 move = float3.zero;

                if (!constraintsRead.TryGetFirstValue(index, out constraint, out iterator)) 
                {
                    return;
                }
                PointReadWrite* pReadWritePointA = pReadWritePoints + index;
                float3 positionA = pReadWritePointA->position;

                int count = 0;
                do
                {
                    count++;
                    PointRead* pPointReadB = pReadPoints + constraint.indexB;
                    PointReadWrite* pReadWritePointB = pReadWritePoints + constraint.indexB;

                    float3 positionB = pReadWritePointB->position;
                    var Direction = positionB - positionA;
                    if (math.all(Direction == 0))
                    {
                        continue;
                    }

                    float Distance = math.length(Direction);

                    float originDistance = constraint.length;
                    float Force = Distance - math.clamp(Distance, originDistance * constraint.shrink, originDistance * constraint.stretch);
                    if (Force != 0)
                    {
                        bool IsShrink = Force >= 0.0f;
                        float ConstraintPower;
                        switch (constraint.type)
                        {
                            case ConstraintType.Structural_Vertical:
                                ConstraintPower = IsShrink
                                    ? (pPointReadA->structuralShrinkVertical + pPointReadB->structuralShrinkVertical)
                                    : (pPointReadA->structuralStretchVertical + pPointReadB->structuralStretchVertical);
                                break;
                            case ConstraintType.Structural_Horizontal:
                                ConstraintPower = IsShrink
                                    ? (pPointReadA->structuralShrinkHorizontal + pPointReadB->structuralShrinkHorizontal)
                                    : (pPointReadA->structuralStretchHorizontal + pPointReadB->structuralStretchHorizontal);
                                break;
                            case ConstraintType.Shear:
                                ConstraintPower = IsShrink
                                    ? (pPointReadA->shearShrink + pPointReadB->shearShrink)
                                    : (pPointReadA->shearStretch + pPointReadB->shearStretch);
                                break;
                            case ConstraintType.Bending_Vertical:
                                ConstraintPower = IsShrink
                                    ? (pPointReadA->bendingShrinkVertical + pPointReadB->bendingShrinkVertical)
                                    : (pPointReadA->bendingStretchVertical + pPointReadB->bendingStretchVertical);
                                break;
                            case ConstraintType.Bending_Horizontal:
                                ConstraintPower = IsShrink
                                    ? (pPointReadA->bendingShrinkHorizontal + pPointReadB->bendingShrinkHorizontal)
                                    : (pPointReadA->bendingStretchHorizontal + pPointReadB->bendingStretchHorizontal);
                                break;
                            case ConstraintType.Circumference:
                                ConstraintPower = IsShrink
                                    ? (pPointReadA->circumferenceShrink + pPointReadB->circumferenceShrink)
                                    : (pPointReadA->circumferenceStretch + pPointReadB->circumferenceStretch);
                                break;
                            default:
                                ConstraintPower = 0.0f;
                                break;
                        }

                        float WeightProportion = pPointReadB->mass / (pPointReadA->mass + pPointReadB->mass);

                        float3 Displacement = Direction / Distance * (Force * ConstraintPower);
                        move += Displacement * WeightProportion;
                    }




                } while (constraintsRead.TryGetNextValue(out constraint, ref iterator));
                if (count != 0)
                {
                    pReadWritePointA->deltaPosition += move / count;
                    pReadWritePointA->position += move / count;
                }
            }
        }
        [BurstCompile]
        public struct JobPointToTransform : IJobParallelForTransform
        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public PointRead* pReadPoints;
            [NativeDisableUnsafePtrRestriction]
            public PointReadWrite* pReadWritePoints;
            [ReadOnly]
            public float deltaTime;
            [ReadOnly]
            public float startDampTime;
            [ReadOnly]
            public float worldScale;
            public void Execute(int index, TransformAccess transform)
            {
                PointReadWrite* pReadWritePoint = pReadWritePoints + index;
                PointRead* pReadPoint = pReadPoints + index;

                float3 writePosition = pReadWritePoint->position * worldScale;
                if (pReadPoint->parentIndex != -1)
                {
                    transform.position = writePosition;
                }


                if (pReadPoint->childFirstIndex > -1 &&
                   !(pReadPoint->isFixedPointFreezeRotation && pReadPoint->parentIndex == -1))
                {
                    transform.localRotation = pReadPoint->initialLocalRotation;
                    int childCount = pReadPoint->childLastIndex - pReadPoint->childFirstIndex;
                    if (childCount > 1) return;

                    float3 ToDirection = 0;
                    float3 FromDirection = 0;
                    for (int i = pReadPoint->childFirstIndex; i < pReadPoint->childLastIndex; i++)
                    {
                        var targetChild = pReadWritePoints + i;
                        var targetChildRead = pReadPoints + i;
                        FromDirection += math.normalize(math.mul((quaternion)transform.rotation, targetChildRead->initialLocalPosition));
                        ToDirection += math.normalize(targetChild->position * worldScale - math.lerp(transform.position, writePosition, startDampTime));

                    }

                    Quaternion AimRotation = FromToRotation(FromDirection, ToDirection);
                    transform.rotation = AimRotation * transform.rotation;
                }

            }

            public static quaternion FromToRotation(float3 from, float3 to, float t = 1.0f)
            {
                from = math.normalize(from);
                to = math.normalize(to);

                float cos = math.dot(from, to);
                float angle = math.acos(cos);
                float3 axis = math.cross(from, to);

                if (math.abs(1.0f + cos) < 1e-06f)
                {
                    angle = (float)math.PI;

                    if (from.x > from.y && from.x > from.z)
                    {
                        axis = math.cross(from, new float3(0, 1, 0));
                    }
                    else
                    {
                        axis = math.cross(from, new float3(1, 0, 0));
                    }
                }
                else if (math.abs(1.0f - cos) < 1e-06f)
                {
                    return quaternion.identity;
                }
                return quaternion.AxisAngle(math.normalize(axis), angle * t);
            }
        }
        #endregion
    }
}
