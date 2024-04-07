﻿using UnityEngine;
using UnityEngine.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using System.Collections.Generic;
using Unity.Jobs;
using System;
using ADBRunTimeJobsTable = TKPhysics.Internal.ADBRunTimeJobsTable;

namespace TKPhysics
{
    using Internal;
    using AutomaticDynamicBone.Runtime;
    using System.Linq;
    using static ADBRunTimeJobsTable;

    public unsafe class ADBPhysicsKernel
    {
        static int BatchLength = 64;
        static int ScheduleBatchedLength = 32;
        private JobHandle Hjob;

        private ADBRunTimeJobsTable.ColliderClacAABB colliderCalcAABB;
        private ADBRunTimeJobsTable.ColliderPositionUpdate colliderUpdate;
        private ADBRunTimeJobsTable.PointGetTransform pointGet;
        private ADBRunTimeJobsTable.PointUpdate pointUpdate;
        private ADBRunTimeJobsTable.ConstraintUpdate constraintUpdates;
        private ADBRunTimeJobsTable.ConstraintForceUpdateByPoint constraintForceUpdateByPoint;
        private ADBRunTimeJobsTable.JobPointToTransform pointToTransform;

        private NativeArray<ColliderRead> collidersReadNativeArray;
        private NativeArray<ColliderReadWrite> collidersReadWriteNativeArray;
        private NativeParallelMultiHashMap<int, ConstraintRead> ConstraintReadMultiHashMap;

        private List<ConstraintRead[]> m_constraintList;
        private List<PointRead> m_pointReadList;
        private List<PointReadWrite> m_pointReadWriteList;
        private NativeArray<ConstraintRead> constraintReadList;
        private NativeArray<PointRead> pointReadNativeArray;
        private NativeArray<PointReadWrite> pointReadWriteListNativeArray;
        private TransformAccessArray pointTransformsAccessArray;
        private List<Transform> m_pointTransforms;
        private bool isInitialize = false;


        private NativeList<JobHandle> CompleteHandleArray;
        public ADBPhysicsKernel()
        {
            Hjob = new JobHandle();
            m_constraintList = new List<ConstraintRead[]>();
            m_pointReadList = new List<PointRead>();
            m_pointReadWriteList = new List<PointReadWrite>();
            m_pointTransforms = new List<Transform>();
            CompleteHandleArray = new NativeList<JobHandle>(8, Allocator.Persistent);

            collidersReadNativeArray = new NativeArray<ColliderRead>(ADBRuntimeController.MAXCOLLIDERCOUNT, Allocator.Persistent);
            collidersReadWriteNativeArray = new NativeArray<ColliderReadWrite>(ADBRuntimeController.MAXCOLLIDERCOUNT, Allocator.Persistent);
        }
        internal bool Schedule(float deltaTime, float scale, ref int iteration, Vector3 addForce, ColliderCollisionType colliderCollisionType, bool isOptimize, bool isRunAsync, bool isParallel,
            float startDampTime)
        {
            if (!isInitialize || !Hjob.IsCompleted)
            {
                return false;
            }
            CompleteHandleArray.Clear();

            float oneDivideIteration = 1.0f / iteration;
            pointToTransform.startDampTime = startDampTime;

            constraintForceUpdateByPoint.oneDivideIteration = constraintUpdates.oneDivideIteration = pointUpdate.oneDivideIteration = colliderCalcAABB.oneDivideIteration = pointGet.oneDivideIteration = colliderUpdate.oneDivideIteration= oneDivideIteration;

            pointUpdate.deltaTime = deltaTime;
            pointToTransform.worldScale=  pointGet.worldScale = scale;
            colliderCalcAABB.localScale = 1 / scale;
            pointUpdate.isOptimize = isOptimize;
            pointUpdate.addForcePower = addForce;
            pointUpdate.isCollision = (colliderCollisionType == ColliderCollisionType.Both || colliderCollisionType == ColliderCollisionType.Point);
            constraintUpdates.isCollision = (colliderCollisionType == ColliderCollisionType.Both || colliderCollisionType == ColliderCollisionType.Constraint); ;

            #region LifeCycle

            if (!isRunAsync)
            {
                colliderCalcAABB.Run(pointUpdate.colliderCount);
                pointGet.Schedule(pointTransformsAccessArray).Complete();

                for (int i = 0; i < iteration; i++)
                {
                    pointUpdate.Run(pointReadNativeArray.Length);
                    colliderUpdate.Run(pointUpdate.colliderCount);

                    if (colliderCollisionType == ColliderCollisionType.Constraint || colliderCollisionType == ColliderCollisionType.Both)
                    {
                        constraintUpdates.Run(constraintReadList.Length);
                    }
                    else
                    {
                        constraintForceUpdateByPoint.Run(pointReadNativeArray.Length);
                    }
                }
            }
            else
            {
                Hjob = JobHandle.CombineDependencies(colliderCalcAABB.Schedule(pointUpdate.colliderCount, BatchLength), pointGet.Schedule(pointTransformsAccessArray));
                CompleteHandleArray.Add(Hjob);
                NativeList<JobHandle> HJobs = CompleteHandleArray;

                for (int i = 0; i < iteration; i++)
                {
                    if (isRunAsync && i % ScheduleBatchedLength == 0)
                    {
                        JobHandle.ScheduleBatchedJobs();
                    }

                    if (!isParallel) 
                    {
                       JobHandle hjob1= colliderUpdate.Schedule(pointUpdate.colliderCount, BatchLength, HJobs[HJobs.Length - 1]);
                        JobHandle hjob2=pointUpdate.Schedule(pointReadNativeArray.Length, BatchLength, HJobs[HJobs.Length - 1]);
                        JobHandle hjob3;
                        if (colliderCollisionType == ColliderCollisionType.Constraint || colliderCollisionType == ColliderCollisionType.Both)
                        {
                            hjob3=constraintUpdates.Schedule(constraintReadList.Length, BatchLength, HJobs[HJobs.Length - 1]);
                        }
                        else
                        {
                            hjob3=constraintForceUpdateByPoint.Schedule(pointReadNativeArray.Length, BatchLength, HJobs[HJobs.Length - 1]);
                        }
                        HJobs.Add(JobHandle.CombineDependencies(hjob1, hjob2, hjob3));
                    }
                    else 
                    {
                        colliderUpdate.Schedule(pointUpdate.colliderCount, BatchLength);
                        pointUpdate.Schedule(pointReadNativeArray.Length, BatchLength);
                        if (colliderCollisionType == ColliderCollisionType.Constraint || colliderCollisionType == ColliderCollisionType.Both)
                        {
                            constraintUpdates.Schedule(constraintReadList.Length, BatchLength);
                        }
                        else
                        {
                            constraintForceUpdateByPoint.Schedule(pointReadNativeArray.Length, BatchLength);
                        }
                    }
                }
                Hjob = JobHandle.CombineDependencies(HJobs.AsArray());
            }

            Hjob = pointToTransform.Schedule(pointTransformsAccessArray, Hjob);
            #endregion
            return true;
        }

        internal void SetRuntimeCollider(ColliderRead[] collidersReadArray)
        {
            NativeArray<ColliderRead>.Copy(collidersReadArray,0, collidersReadNativeArray,0, collidersReadArray.Length);

            pointUpdate.colliderCount = collidersReadArray.Length;
            constraintUpdates.colliderCount = collidersReadArray.Length;
        }
        public void SetPointAndConstraintData(ConstraintRead[][] constraintList, PointRead[] pointReadList, PointReadWrite[] pointReadWriteList, Transform[] pointTransformsList)
        {
            int offset = m_pointReadList.Count;
            for (int i = 0; i < pointReadList.Length; i++)
            {
                PointRead pointRead = pointReadList[i];
                if (pointRead.parentIndex != -1)
                {
                    pointRead.parentIndex += offset;
                }
                if (pointRead.childFirstIndex != -1)
                {
                    pointRead.childFirstIndex += offset;
                    pointRead.childLastIndex += offset;
                }
                pointRead.fixedIndex += offset;
                m_pointReadList.Add(pointRead);
            }

            for (int i = 0; i < constraintList.Length; i++)
            {
                var constraintArray = new ConstraintRead[constraintList[i].Length];
                for (int j0 = 0; j0 < constraintList[i].Length; j0++)
                {
                    var constraint = constraintList[i][j0];
                    constraint.indexA += offset;
                    constraint.indexB += offset;
                    constraintArray[j0] = constraint;
                }
                m_constraintList.Add(constraintArray);
            }
            this.m_pointReadWriteList.AddRange(pointReadWriteList);
            for (int i = 0; i < pointTransformsList.Length; i++)
            {
                m_pointTransforms.Add(pointTransformsList[i]);
            }
        }
        public void SetNativeArray()
        {
            pointReadNativeArray = new NativeArray<PointRead>(m_pointReadList.ToArray(), Allocator.Persistent);
            pointReadWriteListNativeArray = new NativeArray<PointReadWrite>(m_pointReadWriteList.ToArray(), Allocator.Persistent);

            pointTransformsAccessArray = new TransformAccessArray(m_pointTransforms.ToArray());

            List<ConstraintRead> constraintReadListTarget = new List<ConstraintRead>();
            ConstraintReadMultiHashMap = new NativeParallelMultiHashMap<int, ConstraintRead>(8, Allocator.Persistent);
            for (int i = 0; i < m_constraintList.Count; i++)
            {
                constraintReadListTarget.AddRange(m_constraintList[i]);
                for (int j = 0; j < m_constraintList[i].Length; j++)
                {
                    ConstraintRead temp = m_constraintList[i][j];
                    ConstraintReadMultiHashMap.Add(m_constraintList[i][j].indexA, temp);
                    int exchange = temp.indexA;
                    temp.indexA = temp.indexB;
                    temp.indexB = exchange;
                    ConstraintReadMultiHashMap.Add(m_constraintList[i][j].indexB, temp);
                }
            }
            constraintReadList = new NativeArray<ConstraintRead>(constraintReadListTarget.ToArray(), Allocator.Persistent);

            colliderCalcAABB = new ADBRunTimeJobsTable.ColliderClacAABB();
            colliderUpdate = new ADBRunTimeJobsTable.ColliderPositionUpdate();

            pointGet = new ADBRunTimeJobsTable.PointGetTransform();
            pointUpdate = new ADBRunTimeJobsTable.PointUpdate();
            constraintUpdates = new ADBRunTimeJobsTable.ConstraintUpdate();
            constraintForceUpdateByPoint = new ADBRunTimeJobsTable.ConstraintForceUpdateByPoint();
            pointToTransform = new ADBRunTimeJobsTable.JobPointToTransform();

            pointGet.pReadPoints = (PointRead*)pointReadNativeArray.GetUnsafePtr();
            pointGet.pReadWritePoints = (PointReadWrite*)pointReadWriteListNativeArray.GetUnsafePtr();

            pointUpdate.pReadPoints = (PointRead*)pointReadNativeArray.GetUnsafePtr();
            pointUpdate.pReadWritePoints = (PointReadWrite*)pointReadWriteListNativeArray.GetUnsafePtr();
            pointUpdate.pReadColliders = (ColliderRead*)collidersReadNativeArray.GetUnsafePtr();
            pointUpdate.pReadWriteColliders = (ColliderReadWrite*)collidersReadWriteNativeArray.GetUnsafePtr();

            constraintUpdates.pReadPoints = (PointRead*)pointReadNativeArray.GetUnsafePtr();
            constraintUpdates.pReadWritePoints = (PointReadWrite*)pointReadWriteListNativeArray.GetUnsafePtr();
            constraintUpdates.pConstraintsRead = (ConstraintRead*)constraintReadList.GetUnsafePtr();
            constraintUpdates.pReadColliders= (ColliderRead*)collidersReadNativeArray.GetUnsafePtr();
            constraintUpdates.pReadWriteColliders = (ColliderReadWrite*)collidersReadWriteNativeArray.GetUnsafePtr();

            constraintForceUpdateByPoint.pReadPoints = (PointRead*)pointReadNativeArray.GetUnsafePtr();
            constraintForceUpdateByPoint.pReadWritePoints = (PointReadWrite*)pointReadWriteListNativeArray.GetUnsafePtr();
            constraintForceUpdateByPoint.constraintsRead = ConstraintReadMultiHashMap;

            pointToTransform.pReadPoints = (PointRead*)pointReadNativeArray.GetUnsafePtr();
            pointToTransform.pReadWritePoints = (PointReadWrite*)pointReadWriteListNativeArray.GetUnsafePtr();

            colliderCalcAABB.maxPointRadius = m_pointReadList.Select(x => x.radius).Max();
            colliderCalcAABB.pReadColliders=(ColliderRead*)collidersReadNativeArray.GetUnsafePtr();
            colliderCalcAABB.pReadWriteColliders = (ColliderReadWrite*)collidersReadWriteNativeArray.GetUnsafePtr();
            colliderUpdate.pReadColliders = (ColliderRead*)collidersReadNativeArray.GetUnsafePtr();
            colliderUpdate.pReadWriteColliders = (ColliderReadWrite*)collidersReadWriteNativeArray.GetUnsafePtr();

            isInitialize = true;
        }

        public void restorePoint(float scale)
        {
            Hjob.Complete();

            if (pointTransformsAccessArray.isCreated) 
            {
                ADBRunTimeJobsTable.InitiralizePoint1 initialpoint = new ADBRunTimeJobsTable.InitiralizePoint1
                {
                    pReadPoints = (PointRead*)pointReadNativeArray.GetUnsafePtr(),
                    pReadWritePoints = (PointReadWrite*)pointReadWriteListNativeArray.GetUnsafePtr(),
                    worldScale= scale,
                };
                Hjob= initialpoint.Schedule(pointTransformsAccessArray, Hjob);

                ADBRunTimeJobsTable.InitiralizePoint2 initialpoint2 = new ADBRunTimeJobsTable.InitiralizePoint2
                {
                    pReadPoints = (PointRead*)pointReadNativeArray.GetUnsafePtr(),
                    pReadWritePoints = (PointReadWrite*)pointReadWriteListNativeArray.GetUnsafePtr(),
                    worldScale = scale,
                };
                Hjob=initialpoint2.Schedule(pointTransformsAccessArray, Hjob); 
            }
        }
        public void Dispose()
        {
            isInitialize = false;
            Hjob.Complete();

            pointReadNativeArray.Dispose();
            pointReadWriteListNativeArray.Dispose();
            pointTransformsAccessArray.Dispose();

            constraintReadList.Dispose();
            ConstraintReadMultiHashMap.Dispose();
            collidersReadNativeArray.Dispose();
            collidersReadWriteNativeArray.Dispose();

            CompleteHandleArray.Dispose();
        }

        internal void DrawGizmos()
        {
            for (int i = 0; i < collidersReadNativeArray.Length; i++)
            {
                MinMaxAABB aabb = collidersReadNativeArray[i].AABB;
                Gizmos.DrawWireCube(aabb.Center, aabb.Extents);
            }
            for (int i = 0; i < pointReadWriteListNativeArray.Length; i++)
            {
                Gizmos.DrawWireSphere(pointReadWriteListNativeArray[i].position, 0.01f);
            }
        }
    }
}
